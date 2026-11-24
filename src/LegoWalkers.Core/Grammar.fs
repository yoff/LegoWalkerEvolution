namespace LegoWalkers

open System
open System.Collections.Generic

module Grammar =
    let private zAxis = Vec3.create 0 0 1
    let private at x y z = Vec3.create x y z

    let validateAssembly (a: Assembly) =
        let ids = a.Parts |> List.map _.Id
        let bodies = a.Parts |> List.choose _.Body |> Set.ofList
        let connectors = a.Parts |> List.filter (fun p -> p.Body.IsNone)
        let engages body (source: PartInstance) =
            let lo, hi = Catalog.shaftRange source.Part
            a.Parts
            |> List.filter (fun p -> p.Body = Some body)
            |> List.exists (fun p ->
                (Catalog.get p.Part).Ports |> List.exists (fun port ->
                    let world = Vec3.add p.Position (Rotation.apply p.Rotation port.Position)
                    let delta = Vec3.subtract world source.Position
                    let axis = Rotation.apply p.Rotation port.Axis
                    let axial = Vec3.dot delta source.Rotation.X
                    let compatible =
                        match (Catalog.get source.Part).Connector with
                        | Some FrictionlessPin | Some FrictionPin -> port.Kind = RoundHole
                        | Some Axle -> true
                        | None -> false
                    compatible
                    && abs (Vec3.dot axis source.Rotation.X) > 1.0-1e-8
                    && Vec3.length (Vec3.cross delta source.Rotation.X) < 1e-7
                    && axial >= lo-1e-7 && axial <= hi+1e-7))
        [
            if ids.Length <> (Set.ofList ids).Count then "Part instance IDs are not unique."
            for p in a.Parts do
                let spec = Catalog.get p.Part
                if not (Vec3.finite p.Position && Rotation.valid p.Rotation) then
                    $"{p.Id} has an invalid placement."
                if spec.Connector.IsSome <> p.Body.IsNone then
                    $"{p.Id} has an inconsistent connector/body assignment."
            for j in a.Joints do
                if j.BodyA = j.BodyB || not (bodies.Contains j.BodyA && bodies.Contains j.BodyB) then
                    $"{j.Id} has invalid body references."
                if not (List.contains j.Connector ids) then $"{j.Id} has an unknown source connector."
                else
                    let source = a.Parts |> List.find (fun p -> p.Id = j.Connector)
                    if source.Body.IsSome || abs (Vec3.dot source.Rotation.X j.Axis) < 1.0-1e-8 then
                        $"{j.Id} does not align with its connector shaft."
                    elif not (engages j.BodyA source && engages j.BodyB source) then
                        $"{j.Id} does not engage catalog ports on both bodies."
            for index, left in connectors |> List.indexed do
                for right in connectors |> List.skip (index+1) do
                    if Catalog.shaftsOverlap left.Part left.Position left.Rotation right.Part right.Position right.Rotation then
                        $"Connectors {left.Id} and {right.Id} overlap on the same shaft line."
            if not (bodies.Contains a.Chassis) then "Missing chassis body."
        ]

    let compile kind genome =
        let errors =
            match kind with
            | Walker | Leg -> Kinematics.validate genome
            | _ -> [
                yield! Genome.validate genome
                if genome.Remote || genome.TowWagon then "Electronics and wagons apply only to walker and leg models."
              ]
        if not errors.IsEmpty then Error errors
        else
            let parts = ResizeArray<PartInstance>()
            let joints = ResizeArray<Joint>()
            let gears = ResizeArray<GearMesh>()
            let drives = ResizeArray<Drive>()
            let feet = ResizeArray<Foot>()
            let paths = ResizeArray<FootPath>()
            let add id part color pos rot body =
                Catalog.get part |> ignore
                parts.Add { Id = id; Part = part; Color = color; Position = pos; Rotation = rot; Body = body }
            let structural id part color pos rot body = add id part color pos rot (Some body)
            let connector id part pos rot = add id part 8 pos rot None
            let joint id a b pos source =
                joints.Add { Id = id; BodyA = a; BodyB = b; Position = pos; Axis = zAxis; Connector = source }
            let link id part body color normal start finish localStart =
                let rot = Rotation.beamWithNormal normal (Vec3.subtract finish start)
                let origin = Vec3.subtract start (Rotation.apply rot (at 0 0 localStart))
                structural id part color origin rot body
            let phase = float genome.PhaseQuarterTurns * Math.PI/2.0
            let mutable warnings = []

            match kind with
            | HingeFixture | RigidFixture ->
                let bodyB = if kind = RigidFixture then "chassis" else "link"
                structural "beam-a" "32523.dat" 7 (at 0 0 0) Rotation.identity "chassis"
                structural "beam-b" "32523.dat" 4 (at 0 20 0) Rotation.identity bodyB
                let pin = if kind = RigidFixture then "4459.dat" else "3673.dat"
                let rot = { X = at 0 1 0; Y = at -1 0 0; Z = zAxis }
                connector "pin" pin (at 0 10 0) rot
                if kind = HingeFixture then
                    joints.Add { Id = "hinge"; BodyA = "chassis"; BodyB = "link"; Position = at 0 10 0
                                 Axis = at 0 1 0; Connector = "pin" }
            | MotorFixture ->
                structural "motor" "58121.dat" 7 Vec3.zero Rotation.identity "chassis"
                structural "gear" "3648b.dat" 4 (at 0 0 -30) Rotation.identity "drive"
                connector "axle" "3705.dat" Vec3.zero (Rotation.shaft 0)
                joint "motor-output" "chassis" "drive" Vec3.zero "axle"
                drives.Add { MotorPart = "motor"; Joint = "motor-output"; Speed = 3.0*float genome.MotorDirection; MaxTorque = 0.4 }
            | Walker | Leg ->
                let pairs = if kind = Leg then 1 else genome.Pairs
                let driveX = 40.0
                let legShaft station = 2 + station*4
                let platform = Platform.build genome pairs phase
                parts.AddRange platform.Parts
                joints.AddRange platform.Joints
                gears.AddRange platform.Gears
                drives.AddRange platform.Drives

                for station in 0..pairs-1 do
                    let shaftIndex = legShaft station
                    let x = driveX + float shaftIndex*60.0
                    let sides = if kind = Leg then [0] else [0; 1]
                    for side in sides do
                        let sign, face = if side = 0 then -1.0, 0.0 else 1.0, 120.0
                        let legId = $"leg-{station}-{side}"
                        let angle = phase + float ((station+side)%2)*Math.PI
                        let pose =
                            match Kinematics.solve genome angle with
                            | Ok value -> value
                            | Error e -> invalidOp e
                        let point plane (p: Vec3) = at (x+p.X) p.Y (face+sign*plane)
                        let crankBody = $"shaft-{shaftIndex}"
                        let couplerBody, rockerBody = $"{legId}-coupler", $"{legId}-rocker"
                        let normal = Vec3.scale -sign zAxis
                        link $"{legId}-crank" "6632.dat" crankBody 14 normal (point 15 pose.A) (point 15 pose.B) 0
                        link $"{legId}-coupler-part" "32063.dat" couplerBody 1 normal (point 25 pose.B) (point 25 pose.Foot) -50
                        link $"{legId}-rocker-part" "32063.dat" rockerBody 2 normal (point 15 pose.D) (point 15 pose.RockerTip) -50
                        let rockerPin = $"{legId}-ground-pin"
                        let pinRotation sign =
                            { X = at 0 0 sign; Y = at 0 1 0; Z = at -sign 0 0 }
                        connector rockerPin "32002.dat" (point 10 pose.D) (pinRotation sign)
                        joint $"{legId}-ground" "chassis" rockerBody (point 10 pose.D) rockerPin
                        for suffix, p, a, b in ["crank", pose.B, crankBody, couplerBody; "coupler", pose.C, couplerBody, rockerBody] do
                            let pinId = $"{legId}-{suffix}-pin"
                            connector pinId "32002.dat" (point 20 p) (pinRotation -sign)
                            joint $"{legId}-{suffix}" a b (point 20 p) pinId
                        feet.Add { Id = legId; Body = couplerBody; Position = point 25 pose.Foot; Part = $"{legId}-coupler-part" }
                        let path = Kinematics.footPath genome angle
                        paths.Add { path with Id = legId; Points = path.Points |> List.map (point 25) }
                warnings <- [
                    "Candidate mechanism, not a certified walking or physically assembled design."
                    "Only crank/rocker self-clearance is sampled; complete swept-volume and assembly-order checks remain necessary."
                    "Gear constraints idealize teeth, backlash and shaft torsion; 24T idlers are indexed by half a tooth."
                    "The motor is rear-mounted and the transmission is 1:1; balance and real motor loading require evaluation."
                    if genome.TowWagon then
                        "The walker and wagon are separate bodies connected by one beam with two passive transverse pitch hinges; no steering."
                        "Wagon wheels rotate independently on supported 8L stub axles; tire hulls and masses are approximate."
                    if genome.Remote then
                        "8881/8884 payload masses, centers and mount stiffness are estimates; AA cells are included in the mass profile."
                    else
                        "Wired variant assumes ideal external power; tether forces are not modeled."
                ]

            // Put the motor's axle first so upstream motor inference selects the actual output joint.
            let ordered =
                parts |> Seq.sortBy (fun p ->
                    match (Catalog.get p.Part).Connector with
                    | None -> 0
                    | Some Axle -> if p.Id = "axle-0" || p.Id = "axle" then 1 else 2
                    | Some FrictionPin -> 3
                    | Some FrictionlessPin -> 4) |> Seq.toList
            let assembly = {
                Kind = kind; Genome = genome; Parts = ordered; Joints = List.ofSeq joints
                Gears = List.ofSeq gears; Drives = List.ofSeq drives; Feet = List.ofSeq feet
                FootPaths = List.ofSeq paths; Chassis = "chassis"; Warnings = warnings
            }
            match validateAssembly assembly with
            | [] -> Ok assembly
            | errors -> Error errors

    let sweep seed =
        let random = Random seed
        [
            for ground in [3; 4] do
                for coupler in [3; 4] do
                    for rocker in [3; 4] do
                        for phase in 0..3 ->
                            { Genome.defaults with GroundHoles = ground; CouplerHoles = coupler
                                                   RockerHoles = rocker; PhaseQuarterTurns = phase; Seed = seed }
        ]
        |> List.map (fun g -> random.Next(), g)
        |> List.sortBy fst
        |> List.map snd
