frame base:{ }

Prefix: "l_" 
Include: <./rai-robotModels/panda/panda.g>

Prefix: "l2_"
Include: <./rai-robotModels/panda/panda.g>

Prefix: False

thrower_base (base): {
 joint: transXYPhi, limits: [-10,10,-10,10,-4,4],
 shape: ssCylinder, size: [.001, .001, .001], contact: 0
}

Edit l_panda_base: { X: "t(0 2 .05)"}

start_area(base): { shape: box, size: [1, 1, 0.01], color: [0, 0, 1], Q: [0.2, 2, 0.03, 1, 0, 0, 0]}
cargo(start_area): { Q: "t(0.45 0 0.03)", shape: ssBox, size: [.06, .06, .06, 0.01 ], color: [0, 1, 0], mass: .1, contact: 1 , friction: .1}

vis(l_gripper): { shape: marker, size: [0.25] }

Edit l2_panda_base: {X: "t(2.7 2 0.05) d(180 0 0 1)"}

bin (l2_gripper): {shape: ssBox, Q: [0.21, 0, -0.27], size: [0.5, 0.5, 0.06, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [0.21, 0.0, 0.08], size: [0.08, 0.5, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side2 (bin): {shape: ssBox, Q: [0.0, -0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side1 (bin): {shape: ssBox, Q: [-0.21, 0.0, 0.15], size: [0.08, 0.5, 0.3, .02], color: [.6, .6, .6], joint: rigid, friction: .1}
side4 (bin): {shape: ssBox, Q: [0.0, 0.21, 0.08], size: [0.5, 0.08, 0.16, .02], color: [.6, .6, .6], joint: rigid, friction: .1}

camera1(start_area):{
    shape: marker, size: [0.1],
    focalLength: 0.895, width: 640, height: 360, zRange: [0.5, 100]
    Q: "t(-3.5 -0.5 2.08) d(-120 1 0 0) d(45 0 1 0)"
}

camera2(start_area):{
    shape: marker, size: [0.1],
    focalLength: 0.895, width: 640, height: 360, zRange: [0.5, 100]
    Q: "t(3.5 -0.5 2.08) d(-120 1 0 0) d(-45 0 1 0)"
}

camera3(start_area):{
    shape: marker, size: [0.1],
    focalLength: 0.895, width: 640, height: 360, zRange: [0.5, 100]
    Q: "t(3.5 3.0 2.08) d(180 0 1 0) d(60 1 0 0) d(45 0 1 0)"
}

camera4(start_area):{
    shape: marker, size: [0.1],
    focalLength: 0.895, width: 640, height: 360, zRange: [0.5, 100]
    Q: "t(-3.5 3.0 2.08) d(180 0 1 0) d(60 1 0 0) d(-45 0 1 0)"
}