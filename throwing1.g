frame base:{ }

Prefix: "l_" 
Include: <./rai-robotModels/panda/panda.g>

Prefix: False

Edit l_panda_base: { X: "t(0 2 .05)"}
start_area(base): { shape: box, size: [1, 1, 0.01], color: [0, 0, 1], Q: [0.2, 2, 0.03, 1, 0, 0, 0]}
cargo(start_area): { Q: "t(0.45 0 0.48)", shape: ssBox, size: [.06, .06, .06, 0.01 ], color: [0, 1, 0], mass: .1, contact: 1 , friction: .1}

vis(l_gripper): { shape: marker, size: [0.25] }

paddle (start_area): {shape: ssBox, Q: [0.45, 0, 0.43], size: [0.25, 0.25, 0.001, .01], color: [.6, .6, .6], joint: rigid, mass:0.1, contact: 1, friction: .1}
paddle_pick_point(paddle): { shape: marker, size: [0.1], Q:[-0.08, 0, 0] }
side4_p (start_area): {shape: ssBox, Q: [0.45, 0, 0.22], size: [0.1, 0.1, 0.4, .01], color: [.6, .6, .6], joint: rigid, mass:0.1, contact: 1, friction: .1}

bin (base): {shape: ssBox, Q: [2, 2, 0.08], size: [0.5, 0.5, 0.06, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [0.21, 0.0, 0.08], size: [0.08, 0.5, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side2 (bin): {shape: ssBox, Q: [0.0, -0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [-0.21, 0.0, 0.08], size: [0.08, 0.5, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side4 (bin): {shape: ssBox, Q: [0.0, 0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}