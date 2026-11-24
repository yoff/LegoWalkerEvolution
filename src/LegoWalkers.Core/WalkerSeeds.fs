namespace LegoWalkers

open System

module WalkerSeeds =
    let private placement (p: PartInstance) : WalkerPlacement =
        { Id = p.Id; Part = p.Part; Color = p.Color; Position = p.Position; Rotation = p.Rotation }

    let fourBar feet =
        if feet < 2 || feet > Motion.maximumFeet then invalidArg "feet" "A whole-walker seed requires two to six feet."
        let stations = (feet+1)/2
        let phase = Math.PI/2.0
        let platform = Platform.build { Genome.defaults with Remote = true } stations phase
        let bodies, hinges, outputs = ResizeArray<MotionBody>(), ResizeArray<MotionHinge>(), ResizeArray<MotionFoot>()
        let parts = ResizeArray<PartInstance>(platform.Parts)
        let joints = ResizeArray<Joint>(platform.Joints)
        let groundSites = ResizeArray<MotionSite>()
        let last = 2+(stations-1)*4
        for i in 0..last do
            let x = 40.0+float i*60.0
            let angle = if i%2 = 0 then phase else Math.PI/24.0-phase
            groundSites.Add (Motion.site $"bearing-{i}" x 0)
            bodies.Add (Motion.rigid $"shaft-{i}" x 0 angle [Motion.site "root" 0 0])
            hinges.Add (Motion.couple $"bearing-{i}" (Motion.port "chassis" $"bearing-{i}") (Motion.port $"shaft-{i}" "root"))
        for n in 0..feet-1 do
            let station, side = n/2, n%2
            let shaft = $"shaft-{2+station*4}"
            let x = 160.0+float station*240.0
            let id = $"leg-{station}-{side}"
            let offset = float ((station+side)%2)*Math.PI
            let angle = phase+offset
            let program = MotionExamples.fourBar Genome.defaults angle |> Result.defaultWith failwith
            let coupler, rocker = $"{id}-coupler", $"{id}-rocker"
            let crankPort = $"tip-{id}"
            let shaftIndex = bodies |> Seq.findIndex (fun b -> b.Id = shaft)
            let shaftBody = bodies[shaftIndex]
            bodies[shaftIndex] <- { shaftBody with Sites = shaftBody.Sites @ [Motion.site crankPort (20.0*cos offset) (20.0*sin offset)] }
            for original, name in ["coupler", coupler; "rocker", rocker] do
                let b = program.Bodies |> List.find (fun b -> b.Id = original)
                bodies.Add { b with Id = name; Initial = { b.Initial with Origin = Vec3.add b.Initial.Origin (Vec3.create x 0 0) } }
            groundSites.Add (Motion.site $"{id}-ground" (x+float Genome.defaults.GroundHoles*20.0) 0)
            hinges.Add (Motion.couple $"{id}-ground" (Motion.port "chassis" $"{id}-ground") (Motion.port rocker "root"))
            hinges.Add (Motion.couple $"{id}-crank" (Motion.port shaft crankPort) (Motion.port coupler "root"))
            hinges.Add (Motion.couple $"{id}-coupler" (Motion.port coupler "joint") (Motion.port rocker "joint"))
            outputs.Add (Motion.foot id coupler "foot" 10)
            let sign, face = if side = 0 then -1.0, 0.0 else 1.0, 120.0
            let at depth (p: Vec3) = Vec3.create (x+p.X) p.Y (face+sign*depth)
            let normal = Vec3.create 0 0 -sign
            let link partId part owner color depth (b: MotionBody) start finish localStart =
                let a = Motion.transform b.Initial start |> at depth
                let z = Motion.transform b.Initial finish |> at depth
                let rotation = Rotation.beamWithNormal normal (Vec3.subtract z a)
                parts.Add { Id = partId; Part = part; Color = color; Body = Some owner; Rotation = rotation
                            Position = Vec3.subtract a (Rotation.apply rotation (Vec3.create 0 0 localStart)) }
            let original id = program.Bodies |> List.find (fun b -> b.Id = id)
            link $"{id}-crank" "6632.dat" shaft 14 15 (original "crank") Vec3.zero (Vec3.create 20 0 0) 0
            link $"{id}-coupler-part" "32063.dat" coupler 1 25 (original "coupler") Vec3.zero (Vec3.create 100 0 0) -50
            link $"{id}-rocker-part" "32063.dat" rocker 2 15 (original "rocker") Vec3.zero (Vec3.create 100 0 0) -50
            let pin group depth direction point a b =
                let rotation = { X = Vec3.create 0 0 direction; Y = Vec3.create 0 1 0; Z = Vec3.create -direction 0 0 }
                let pos = at depth point
                parts.Add { Id = $"pivot-{group}"; Part = "32002.dat"; Color = 8; Position = pos; Rotation = rotation; Body = None }
                joints.Add { Id = group; BodyA = a; BodyB = b; Position = pos; Axis = Vec3.create 0 0 1; Connector = $"pivot-{group}" }
            let couplerPose = (original "coupler").Initial
            pin $"{id}-ground" 10 sign (original "rocker").Initial.Origin "chassis" rocker
            pin $"{id}-crank" 20 -sign couplerPose.Origin shaft coupler
            pin $"{id}-coupler" 20 -sign (Motion.transform couplerPose (Vec3.create (float Genome.defaults.CouplerHoles*20.0) 0 0)) coupler rocker
        bodies.Add (Motion.ground "chassis" (List.ofSeq groundSites))
        let program = {
            Bodies = List.ofSeq bodies; Hinges = List.ofSeq hinges; Motor = { Body = "shaft-0"; Phase = phase }
            Transmissions = [for i in 1..last -> Motion.transmit $"shaft-{i-1}" $"shaft-{i}" -1 (Math.PI/24.0)]
            Feet = List.ofSeq outputs
        }
        let pivotByPart = joints |> Seq.map (fun j -> j.Connector, j) |> Map.ofSeq
        let genome = {
            SchemaVersion = 1; RecipeVersion = WalkerRealization.recipeVersion; MechanismId = Realization.mechanismId program
            Remote = true; TowWagon = false
            Bodies = parts |> Seq.choose (fun p -> p.Body |> Option.map (fun b -> b, p)) |> Seq.groupBy fst
                     |> Seq.map (fun (id, ps) -> { Body = id; Parts = ps |> Seq.map (snd >> placement) |> Seq.toList }) |> Seq.toList
            Connections = [
                for p in parts do
                    match Map.tryFind p.Id pivotByPart with
                    | Some j -> { Group = j.Id; Hinges = [j.Id]; Placement = { placement p with Id = $"pivot-{j.Id}" } }
                    | None -> ()
            ]
            Mounts = parts |> Seq.filter (fun p -> p.Body.IsNone && not (pivotByPart.ContainsKey p.Id)) |> Seq.map placement |> Seq.toList
            Feet = [for n in 0..feet-1 ->
                        let id = $"leg-{n/2}-{n%2}"
                        { Foot = id; Part = $"{id}-coupler-part"; Depth = if n%2 = 0 then -25.0 else 145.0
                          AdditionalContacts = None }]
            MotorPart = "motor"; Speed = 3.0; MaxTorque = 0.4; LegFamilies = None
        }
        program, genome

    let sharedFourBar feet =
        let program,genome = fourBar feet
        let instances = [
            for n in 0..feet-1 ->
                let station,side = n/2,n%2
                let id = $"leg-{station}-{side}"
                WalkerLegFamilies.instance id id (side=1) (float ((station+side)%2)*Math.PI) program
        ]
        program,WalkerLegFamilies.bind "legs" instances program genome
