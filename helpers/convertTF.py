import tensorflow as tf

# Load the Keras model
keras_model = tf.keras.models.load_model("smartcart_classifier.h5n")

# Convert to TFLite format
converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
tflite_model = converter.convert()

# Save TFLite model
with open("smartcart_classifier.tflite", "wb") as f:
    f.write(tflite_model)

print("✅ Model converted to TensorFlow Lite!")