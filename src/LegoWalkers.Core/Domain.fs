namespace LegoWalkers

open System

[<Struct>]
type Vec3 = { X: float; Y: float; Z: float }

module Vec3 =
    let create x y z = { X = x; Y = y; Z = z }
    let zero = create 0.0 0.0 0.0
    let add a b = create (a.X + b.X) (a.Y + b.Y) (a.Z + b.Z)
    let subtract a b = create (a.X - b.X) (a.Y - b.Y) (a.Z - b.Z)
    let scale k a = create (k * a.X) (k * a.Y) (k * a.Z)
    let dot a b = a.X * b.X + a.Y * b.Y + a.Z * b.Z
    let cross a b = create (a.Y*b.Z-a.Z*b.Y) (a.Z*b.X-a.X*b.Z) (a.X*b.Y-a.Y*b.X)
    let length a = sqrt (dot a a)
    let distance a b = length (subtract a b)
    let normalize a =
        let n = length a
        if n < 1e-12 then invalidArg (nameof a) "Cannot normalize a zero vector."
        scale (1.0 / n) a
    let finite a = Double.IsFinite a.X && Double.IsFinite a.Y && Double.IsFinite a.Z

/// Columns of a proper rotation; all assembly positions are in LDraw units.
type Rotation = { X: Vec3; Y: Vec3; Z: Vec3 }

module Rotation =
    let identity = { X = Vec3.create 1 0 0; Y = Vec3.create 0 1 0; Z = Vec3.create 0 0 1 }
    let apply (r: Rotation) (p: Vec3) =
        Vec3.add (Vec3.scale p.X r.X) (Vec3.add (Vec3.scale p.Y r.Y) (Vec3.scale p.Z r.Z))
    let aroundZ angle =
        let c, s = cos angle, sin angle
        { X = Vec3.create c s 0; Y = Vec3.create -s c 0; Z = Vec3.create 0 0 1 }
    let shaft angle =
        let c, s = cos angle, sin angle
        { X = Vec3.create 0 0 1; Y = Vec3.create c s 0; Z = Vec3.create -s c 0 }
    let beamWithNormal normal direction =
        let z = Vec3.normalize direction
        let y = Vec3.normalize normal
        { X = Vec3.cross y z; Y = y; Z = z }
    let beam direction = beamWithNormal (Vec3.create 0 0 1) direction
    let valid r =
        let near a b = abs (a-b) < 1e-8
        [r.X; r.Y; r.Z] |> List.forall (fun v -> Vec3.finite v && near (Vec3.length v) 1.0)
        && near (Vec3.dot r.X r.Y) 0.0
        && near (Vec3.dot r.X r.Z) 0.0
        && near (Vec3.dot r.Y r.Z) 0.0
        && near (Vec3.dot r.X (Vec3.cross r.Y r.Z)) 1.0

type PortKind = RoundHole | AxleHole
type ConnectorRecipe = FrictionPin | FrictionlessPin | Axle
type LocalPort = { Position: Vec3; Axis: Vec3; Kind: PortKind }
type PartSpec = { Id: string; Connector: ConnectorRecipe option; Ports: LocalPort list }

module Catalog =
    let private port kind x y z axis = { Position = Vec3.create x y z; Axis = axis; Kind = kind }
    let private y = Vec3.create 0 1 0
    let private z = Vec3.create 0 0 1
    let private spec id connector ports = { Id = id; Connector = connector; Ports = ports }
    let parts =
        [
            spec "32523.dat" None [for n in -1..1 -> port RoundHole 0 0 (float n*20.0) y]
            spec "60483.dat" None [port AxleHole 0 0 0 y; port RoundHole 0 0 20 y]
            spec "32316.dat" None [for n in -2..2 -> port RoundHole 0 0 (float n*20.0) y]
            spec "32524.dat" None [for n in -3..3 -> port RoundHole 0 0 (float n*20.0) y]
            spec "32525.dat" None [for n in -5..5 -> port RoundHole 0 0 (float n*20.0) y]
            spec "32526.dat" None [
                for n in 0..4 -> port RoundHole 0 0 (float n*20.0) y
                for n in 1..2 -> port RoundHole (float n*20.0) 0 80 y
            ]
            spec "32271.dat" None [
                yield port AxleHole 0 0 0 y
                for n in 1..6 -> port RoundHole 0 0 (float n*20.0) y
                yield port RoundHole 16 0 132 y
                yield port AxleHole 32 0 144 y
            ]
            spec "3701.dat" None [for n in -1..1 -> port RoundHole (float n*20.0) 10 0 z]
            spec "3710.dat" None []
            spec "32063.dat" None [for n in 0..5 -> port RoundHole 0 0 (float n*20.0-50.0) y]
            spec "6632.dat" None [
                port AxleHole 0 0 0 y
                port RoundHole 0 0 20 y
                port AxleHole 0 0 40 y
            ]
            spec "3703.dat" None [for n in -7..7 -> port RoundHole (float n*20.0) 10 0 z]
            spec "3648b.dat" None [port AxleHole 0 0 0 z]
            spec "3647.dat" None [port AxleHole 0 0 0 z]
            spec "3649.dat" None [port AxleHole 0 0 0 z]
            spec "3713.dat" None [port AxleHole 0 0 0 z]
            spec "4265c.dat" None [port AxleHole 0 0 0 z]
            spec "56145c01.dat" None [port AxleHole 0 0 0 z]
            spec "58121.dat" None [
                yield port AxleHole 0 0 0 z
                yield port AxleHole 0 0 20 z
                for x in [-20.0; 20.0] -> port RoundHole x 0 0 z
                for depth in [10.0;110.0] do
                    for side in [-1.0;1.0] -> port RoundHole (side*50.0) 0 depth (Vec3.create side 0 0)
            ]
            spec "6536.dat" None [
                port AxleHole 0 0 0 (Vec3.create 1 0 0)
                port RoundHole 0 20 0 z
            ]
            spec "3460.dat" None []
            spec "58119.dat" None [
                for x in [-100.0; 100.0] do
                    for z in [-30.0; 30.0] -> port RoundHole x 0 z (Vec3.create 0 0 1)
            ]
            spec "58123.dat" None [
                for x in [-20.0; 20.0] -> port RoundHole x 10 -40 (Vec3.create 0 0 1)
            ]
            spec "4459.dat" (Some FrictionPin) []
            spec "6558.dat" (Some FrictionPin) []
            spec "3673.dat" (Some FrictionlessPin) []
            spec "32002.dat" (Some FrictionlessPin) []
            spec "32556.dat" (Some FrictionlessPin) []
            spec "3705.dat" (Some Axle) []
            spec "3707.dat" (Some Axle) []
            spec "3708.dat" (Some Axle) []
            spec "3737.dat" (Some Axle) []
            spec "44294.dat" (Some Axle) []
            spec "60485.dat" (Some Axle) []
        ] |> List.map (fun p -> p.Id, p) |> Map.ofList
    let get id =
        match Map.tryFind id parts with
        | Some part -> part
        | None -> invalidArg (nameof id) $"Unsupported part: {id}"

    let soleUnderside = function
        | "3701.dat" -> Some 24.0
        | "3710.dat" -> Some 8.0
        | _ -> None

    let mountPorts id =
        (get id).Ports |> List.filter (fun p ->
            p.Kind=RoundHole &&
            (id<>"58121.dat" || (abs p.Axis.X>0.999 && abs(p.Position.Z-10.0)<1e-6)))

    let planarBeamParts = ["60483.dat";"32523.dat";"32316.dat";"32063.dat";"32524.dat";"32525.dat"]
    let axleParts = ["3705.dat";"44294.dat";"3707.dat";"60485.dat";"3737.dat";"3708.dat"]

    /// Proper rigid placements matching every requested socket; transverse axes may face either way.
    let fitPlanarPart id (axis: Vec3) (points: Vec3 list) =
        let spec = get id
        let near a b = Vec3.distance a b < 1e-6
        let points = List.distinct points
        if not (Vec3.finite axis) || abs (Vec3.length axis-1.0) > 1e-8 then
            invalidArg (nameof axis) "Socket axis must be finite and unit length."
        if points.Length < 2 || points |> List.exists (Vec3.finite >> not) then
            invalidArg (nameof points) "At least two distinct finite socket positions are required."
        let target = Vec3.subtract points[1] points[0]
        let length = Vec3.length target
        if length < 1e-6 || points |> List.exists (fun p -> abs (Vec3.dot axis (Vec3.subtract p points[0])) > 1e-6) then
            invalidArg (nameof points) "Requested sockets must be distinct and coplanar perpendicular to the axis."
        let targetU = Vec3.scale (1.0/length) target
        [for first in spec.Ports do
            for second in spec.Ports do
                let delta = Vec3.subtract second.Position first.Position
                if abs (Vec3.length delta-length) < 1e-6
                   && abs (Vec3.dot delta first.Axis) < 1e-6
                   && abs (abs (Vec3.dot first.Axis second.Axis)-1.0) < 1e-8 then
                    let u = Vec3.normalize delta
                    let v = Vec3.cross first.Axis u
                    for normal in [axis; Vec3.scale -1.0 axis] do
                        let targetV = Vec3.cross normal targetU
                        let map p =
                            Vec3.add (Vec3.scale (Vec3.dot p u) targetU)
                                (Vec3.add (Vec3.scale (Vec3.dot p v) targetV) (Vec3.scale (Vec3.dot p first.Axis) normal))
                        let rotation = { X=map (Vec3.create 1 0 0); Y=map (Vec3.create 0 1 0); Z=map (Vec3.create 0 0 1) }
                        let position = Vec3.subtract points[0] (Rotation.apply rotation first.Position)
                        let matches point =
                            spec.Ports |> List.exists (fun hole ->
                                near point (Vec3.add position (Rotation.apply rotation hole.Position))
                                && abs (abs (Vec3.dot axis (Rotation.apply rotation hole.Axis))-1.0) < 1e-8)
                        if Rotation.valid rotation && points |> List.forall matches then yield position,rotation]
        |> List.distinct
    let gearTeeth = function
        | "3647.dat" -> Some 8
        | "3648b.dat" -> Some 24
        | "3649.dat" -> Some 40
        | _ -> None
    let matchesSpurMesh ratio partA (positionA: Vec3) partB (positionB: Vec3) =
        match gearTeeth partA,gearTeeth partB with
        | Some a,Some b ->
            abs(ratio+float a/float b)<1e-6
            && abs(positionA.Z-positionB.Z)<1e-6
            && abs(Vec3.distance positionA positionB-float(a+b)*1.25)<1e-6
        | _ -> false
    let shaftRange = function
        | "32002.dat" -> -20.0, 10.0
        | "6558.dat" -> -30.0, 30.0
        | "32556.dat" -> -30.0, 30.0
        | "4459.dat" | "3673.dat" -> -20.0, 20.0
        | "3705.dat" -> -40.0, 40.0
        | "3707.dat" -> -80.0, 80.0
        | "3708.dat" -> -120.0, 120.0
        | "3737.dat" -> -100.0, 100.0
        | "44294.dat" -> -70.0, 70.0
        | "60485.dat" -> -90.0, 90.0
        | id -> invalidArg (nameof id) $"Part is not a supported shaft connector: {id}"

    let socketHalfThickness part =
        if part="32063.dat" || part="6632.dat" then 5.0 else 10.0

    let shaftsOverlap part (position: Vec3) (rotation: Rotation) otherPart (otherPosition: Vec3) (otherRotation: Rotation) =
        let delta = Vec3.subtract otherPosition position
        let alignment = Vec3.dot rotation.X otherRotation.X
        if abs alignment <= 1.0-1e-8 || Vec3.length (Vec3.cross delta rotation.X) >= 1e-7 then false
        else
            let lo,hi = shaftRange part
            let otherLo,otherHi = shaftRange otherPart
            let offset = Vec3.dot delta rotation.X
            let ends = [offset+otherLo*alignment;offset+otherHi*alignment]
            min hi (List.max ends)-max lo (List.min ends) > 1e-7

type Genome = {
    Pairs: int
    GroundHoles: int
    CouplerHoles: int
    RockerHoles: int
    PhaseQuarterTurns: int
    Seed: int
    Remote: bool
    BatterySlot: int
    BatteryLevel: int
    ReceiverSlot: int
    ReceiverSide: int
    MotorDirection: int
    TowWagon: bool
    BatteryOnWagon: bool
    ReceiverOnWagon: bool
    WagonAxleHole: int
}

module Genome =
    let defaults = {
        Pairs = 3; GroundHoles = 4; CouplerHoles = 4; RockerHoles = 4
        PhaseQuarterTurns = 1; Seed = 0; Remote = false
        BatterySlot = 1; BatteryLevel = 0; ReceiverSlot = 0; ReceiverSide = 1; MotorDirection = 1
        TowWagon = false; BatteryOnWagon = false; ReceiverOnWagon = false; WagonAxleHole = 4
    }
    let validate g = [
        if g.Pairs < 1 || g.Pairs > 3 then "pairs must be between 1 and 3."
        for name, value in ["ground", g.GroundHoles; "coupler", g.CouplerHoles; "rocker", g.RockerHoles] do
            if value <> 3 && value <> 4 then $"{name} must select a 3- or 4-hole span (60 or 80 LDU)."
        if g.PhaseQuarterTurns < 0 || g.PhaseQuarterTurns > 3 then "phase must be a quarter-turn index from 0 to 3."
        for name, value in ["battery slot", g.BatterySlot; "receiver slot", g.ReceiverSlot] do
            if value < 0 || value > 2 then $"{name} must be 0, 1 or 2."
        if g.BatteryLevel < 0 || g.BatteryLevel > 1 then "battery level must be 0 or 1."
        if g.ReceiverSide < 0 || g.ReceiverSide > 1 then "receiver side must be 0 or 1."
        if abs g.MotorDirection <> 1 then "motor direction must be -1 or 1."
        if g.WagonAxleHole < 2 || g.WagonAxleHole > 4 then "wagon axle hole must be 2, 3 or 4."
        if not g.TowWagon && g.WagonAxleHole <> 4 then "Solo genomes must use the canonical inactive wagon axle hole."
        if (g.BatteryOnWagon || g.ReceiverOnWagon) && (not g.Remote || not g.TowWagon) then
            "Wagon electronics placement requires a remote towing design."
        if not g.Remote && (g.BatterySlot <> 1 || g.BatteryLevel <> 0 || g.ReceiverSlot <> 0 || g.ReceiverSide <> 1) then
            "Wired genomes must use canonical inactive payload genes."
    ]

type ModelKind = Walker | Leg | HingeFixture | RigidFixture | MotorFixture

module ModelKind =
    let name = function
        | Walker -> "walker"
        | Leg -> "leg"
        | HingeFixture -> "hinge"
        | RigidFixture -> "rigid"
        | MotorFixture -> "motor"
    let parse = function
        | "walker" -> Walker | "leg" -> Leg | "hinge" -> HingeFixture
        | "rigid" -> RigidFixture | "motor" -> MotorFixture
        | value -> invalidArg "kind" $"Unknown model kind: {value}"

type PartInstance = {
    Id: string
    Part: string
    Color: int
    Position: Vec3
    Rotation: Rotation
    Body: string option
}

type Joint = {
    Id: string
    BodyA: string
    BodyB: string
    Position: Vec3
    Axis: Vec3
    Connector: string
}

type GearMesh = { BodyA: string; BodyB: string; Ratio: float }
type Drive = { MotorPart: string; Joint: string; Speed: float; MaxTorque: float }
type Foot = { Id: string; Body: string; Position: Vec3; Part: string }
type FootPath = { Id: string; Points: Vec3 list; Stride: float; Lift: float }

type Assembly = {
    Kind: ModelKind
    Genome: Genome
    Parts: PartInstance list
    Joints: Joint list
    Gears: GearMesh list
    Drives: Drive list
    Feet: Foot list
    FootPaths: FootPath list
    Chassis: string
    Warnings: string list
}
