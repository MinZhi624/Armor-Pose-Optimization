# Model Directory

Place `number_cnn.onnx` here before running the detector.

The ONNX model expects:
- Input: 1x1x28x28 grayscale image, normalized to [0,1]
- Output: 1x11 tensor
  - [0]: objectness logit
  - [1..10]: 10 class logits (softmax applied internally)
