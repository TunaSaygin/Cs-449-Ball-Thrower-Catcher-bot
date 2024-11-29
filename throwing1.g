frame base:{ }

Prefix: "t_" 
Include: <./rai-robotModels/panda/panda.g>

Prefix: False

Edit t_panda_base: { X: "t(0 2 .05)"}
start_area(base): { shape: box, size: [1, 1, 0.01], color: [0, 0, 1], Q: [0.2, 2, 0.03, 1, 0, 0, 0] ,contact: -2}
cargo(start_area): { Q: "t(0.2 0. 0.06)", shape: sphere, size: [.06 ], color: [0, 1, 0], mass: .1, contact: 1 , friction: .1}

bin (base): {shape: ssBox, Q: [0.0, -0.5, 0.08], size: [0.5, 0.5, 0.06, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [0.21, 0.0, 0.08], size: [0.08, 0.5, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side2 (bin): {shape: ssBox, Q: [0.0, -0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [-0.21, 0.0, 0.08], size: [0.08, 0.5, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side4 (bin): {shape: ssBox, Q: [0.0, 0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}