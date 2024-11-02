frame base:{ }

Prefix: "t_" 
Include: <../rai-robotModels/panda/panda.g>

Prefix: False

Edit t_panda_base: { X: "t(0 2 .05)"}
start_area(base): { shape: box, size: [1, 1, 0.01], color: [0, 0, 1], Q: [.8, 2, 0.03, 1, 0, 0, 0] ,contact: -2}
cargo: { X: "t(.6 2. 0.13)", shape: sphere, size: [.1 ], color: [0, 1, 0], mass: .1, contact: 1 }
camera: { X: "t(.6 -2. 0.5)", shape: camera, size: [.8], color: [0, 1, 0], contact: 1 }
Prefix: "c_" 
Include: <../rai-robotModels/panda/panda.g>

Prefix: False

Edit c_panda_base: { X: "t(0 -2 .05)" }

goal_area(base): { shape: box, size: [1.5, 1.5, 0.01], color: [1, 0, 0], Q: [0, -2, 0.03, 1, 0, 0, 0] ,contact: -2}

place_goal: {X: "t(0.6 -2.2 0.13) d(90 0 0 1)", shape: marker,size: [.1], contact: 0}
