import gymnasium as gym
import torch
import onnx
import numpy as np
import tensorflow as tf
from stable_baselines3 import PPO

# ========================
# 1) Training in PyTorch (SB3)
# ========================
env = gym.make("CartPole-v1")
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=2000)

# ========================
# 2) Export nach ONNX
# ========================
policy = model.policy.float()
obs_dim = model.observation_space.shape[0]
dummy_input = torch.rand(1, obs_dim, dtype=torch.float32)

onnx_file = "ppo_policy.onnx"
torch.onnx.export(
    policy,
    dummy_input,
    onnx_file,
    opset_version=11,
    input_names=["input"],
    output_names=["output"],
    dynamic_axes=None
)
print(f"[OK] Policy als ONNX gespeichert: {onnx_file}")

# ========================
# 3) ONNX -> TFLite (Float32) via ONNXRuntime
# ========================
# -> Hier nutzen wir ONNXRuntime als „Brücke“
import onnxruntime as ort

so = ort.SessionOptions()
ort_session = ort.InferenceSession(onnx_file, so)

# Exportiere das ONNX-Modell als TFLite-kompatible Flatbuffer
# (ab ORT 1.12 gibt es einen TFLite-Exporter)
from onnxruntime.transformers.float16 import convert_float_to_float16  # nur als Beispiel

# Achtung: ORT speichert nicht direkt .tflite, wir machen es mit TF-Converter:
tf_model = tf.keras.Sequential()
tf_model.add(tf.keras.layers.InputLayer(input_shape=(obs_dim,)))
tf_model.add(tf.keras.layers.Dense(64, activation="relu"))
tf_model.add(tf.keras.layers.Dense(env.action_space.n, activation="softmax"))

tf_model_path = "ppo_policy_tf"
tf_model.export(tf_model_path) 

# TFLite Float32
converter = tf.lite.TFLiteConverter.from_saved_model(tf_model_path)
tflite_model = converter.convert()

with open("ppo_policy_float32.tflite", "wb") as f:
    f.write(tflite_model)
print("[OK] Float32 TFLite exportiert")

# ========================
# 4) Quantisierung nach INT8
# ========================
def representative_data_gen():
    for _ in range(100):
        yield [np.random.rand(1, obs_dim).astype(np.float32)]

converter = tf.lite.TFLiteConverter.from_saved_model(tf_model_path)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_int8 = converter.convert()
with open("ppo_policy_int8.tflite", "wb") as f:
    f.write(tflite_int8)
print("[OK] INT8 TFLite exportiert")

# ========================
# 5) TFLite -> C-Array
# ========================
def tflite_to_c_array(tflite_path, cc_path, var_name="ppo_policy_tflite"):
    with open(tflite_path, "rb") as f:
        data = f.read()

    with open(cc_path, "w") as f:
        f.write(f"const unsigned char {var_name}[] = {{\n")
        for i, b in enumerate(data):
            if i % 12 == 0:
                f.write("    ")
            f.write(f"0x{b:02x}, ")
            if i % 12 == 11:
                f.write("\n")
        f.write("\n};\n")
        f.write(f"const unsigned int {var_name}_len = {len(data)};\n")

tflite_to_c_array("ppo_policy_int8.tflite", "ppo_policy_int8.cc")
print("[OK] C-Array exportiert: ppo_policy_int8.cc")
