# MoE Rough policy provenance

- Project: `wty-yy/go2_rl_gym`
- Revision: `30e74dc507bec7a642a8c98be26081f2c6f0822d`
- Source artifact: `deploy/pre_train/go2/go2_moe_cts_high_slope_thre_164k_0.6715.pt`
- Source SHA-256: `9d9ad783a1017b6eced5984eb95279cc5b36db8cc84d21e646f46ba2a8023d9d`
- Converted artifact: `policy.onnx`, opset 17, explicit frame-major `5 x 45` history
- Converted SHA-256: `1d9e76089401a011f38d6e81708f7334618e9faaad078999a1f6d94b07381f1d`
- Conversion parity: maximum absolute action error `1.02e-6` over a fixed five-frame input
- License: upstream additions are MIT; inherited Unitree components are BSD-3-Clause

The browser runner follows the upstream MuJoCo deployment scales, joint order,
50 Hz policy rate, 20/0.5 PD gains, and 0.25 action scale. The automatic stair
round trip is a project-local high-level command generator and does not modify
the neural network.
