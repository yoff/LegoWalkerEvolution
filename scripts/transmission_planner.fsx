#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open System.Security.Cryptography
open System.Text.Json
open LegoWalkers

type DriveNode = { Id: string; Position: Vec3; Phase: float; Rate: float }
type DriveEdge = { Input: string; Output: string; Phase: float }
type DriveRoute = { Root: string; Nodes: DriveNode list; Edges: DriveEdge list }
type DriveGear = { Placement: PartInstance; Teeth: int; Meshes: string list }
type DriveShaft = {
    Placement: PartInstance; SocketCentersZ: float list
    BearingTargets: Vec3 list; MinimumReachZ: float; MaximumReachZ: float
}
type DriveComparison = {
    SampledPhases: int; MaxBodyPositionErrorLdu: float
    MaxBodyAngleErrorRadians: float; MaxFootPositionErrorLdu: float
    MotorClockScale: float; ReferenceMotorSpeed: float; ConstructedMotorSpeed: float
}
type DriveFrame = {
    ReferenceInputAngle: float; MotorInputAngle: float
    GearAngles: (string * float) list; Feet: Vec3 list
}

module DrivePlanning =
    let pitch = Math.PI/24.0
    let angleError a b = abs (atan2 (sin (a-b)) (cos (a-b)))
    let normalize a = atan2 (sin a) (cos a)
    let lattice value = abs (value/60.0-Math.Round(value/60.0)) < 1e-7

    let route (demands: DriveNode list) (existing: Map<Vec3,string>) (root: Vec3) =
        if demands.IsEmpty then invalidArg "demands" "At least one required rotation is necessary."
        let reference = demands.Head
        if demands |> List.exists (fun node -> abs (node.Position.Y-reference.Position.Y) > 1e-7
                                               || not (lattice (node.Position.X-reference.Position.X))
                                               || abs (abs node.Rate-1.0) > 1e-7) then
            invalidArg "demands" "Prototype requires collinear fixed output axes on a 60 LDU lattice and unit-magnitude rates."
        if not (Vec3.finite root) || abs root.Z > 1e-7
           || not (lattice (root.X-reference.Position.X)) || abs (root.Y-reference.Position.Y) > 60.000001
           || not (lattice (root.Y-reference.Position.Y)) then
            invalidArg "root" "Prototype motor axes must be on the same lattice, at most one mesh above or below it."
        let low = min root.X (demands |> List.minBy _.Position.X).Position.X
        let high = max root.X (demands |> List.maxBy _.Position.X).Position.X
        let positions = [for i in 0..int (Math.Round((high-low)/60.0)) ->
                             Vec3.create (low+float i*60.0) reference.Position.Y 0]
        let positions = if abs (root.Y-reference.Position.Y) > 1e-7 then positions @ [root] else positions
        if positions.Length > 16 then invalidArg "route" "Prototype route exceeds 16 parallel shafts."
        let id position =
            match demands |> List.tryFind (fun node -> Vec3.distance node.Position position < 1e-7) with
            | Some demand -> demand.Id
            | None ->
                existing |> Map.toList |> List.tryFind (fun (p,_) -> Vec3.distance p position < 1e-6)
                |> Option.map snd |> Option.defaultValue $"constructed-shaft-{int position.X}-{int position.Y}"
        let nodes = positions |> List.map (fun p -> id p,p)
        let pairs =
            (positions |> List.filter (fun p -> abs (p.Y-reference.Position.Y) < 1e-7) |> List.pairwise)
            @ (if abs (root.Y-reference.Position.Y) > 1e-7 then
                   [(root,Vec3.create root.X reference.Position.Y 0)] else [])
        let pairs = pairs |> List.map (fun (a,b) -> id a,id b)
        let neighbors current = pairs |> List.choose (fun (a,b) ->
            if a=current then Some b elif b=current then Some a else None) |> List.sort
        let rec assign states = function
            | [] -> states
            | current::rest ->
                let rate,phase = states |> Map.find current
                let fresh = neighbors current |> List.filter (fun next -> not (Map.containsKey next states))
                let states = fresh |> List.fold (fun acc next -> Map.add next (-rate,normalize (pitch-phase)) acc) states
                assign states (rest @ fresh)
        let states = assign (Map.ofList [(reference.Id,(reference.Rate,reference.Phase))]) [reference.Id]
        for demand in demands do
            let rate,phase = states[demand.Id]
            if abs (rate-demand.Rate) > 1e-7 || angleError phase demand.Phase > 1e-6 then
                invalidArg "demands" $"Required rate/phase at {demand.Id} cannot be reproduced by this 24T route."
        let rec orient visited edges = function
            | [] -> edges
            | current::rest ->
                let fresh = neighbors current |> List.filter (fun next -> not (Set.contains next visited))
                let visited = fresh |> List.fold (fun acc next -> Set.add next acc) visited
                let edges = edges @ (fresh |> List.map (fun next ->
                    { Input=current; Output=next; Phase=snd states[current]+snd states[next] }))
                orient visited edges (rest @ fresh)
        let rootId = id root
        let edges = orient (Set.singleton rootId) [] [rootId]
        let degrees id = pairs |> List.filter (fun (a,b) -> a=id || b=id) |> List.length
        let demanded = demands |> List.map _.Id |> Set.ofList
        if nodes |> List.exists (fun (id,_) -> degrees id <= 1 && id <> rootId && not (demanded.Contains id)) then
            invalidOp "Constructed route contains a dead branch."
        { Root=rootId
          Nodes=nodes |> List.map (fun (id,p) -> { Id=id;Position=p;Rate=fst states[id];Phase=snd states[id] })
          Edges=edges }

    let gearUses (route: DriveRoute) =
        route.Nodes |> List.map (fun node ->
            node.Id,route.Edges |> List.filter (fun edge -> edge.Input=node.Id || edge.Output=node.Id)
                               |> List.map (fun edge -> edge.Input+"->"+edge.Output))

let selfTest () =
    let require condition message = if not condition then failwith message
    let p x y = Vec3.create x y 0
    let demand x = { Id = $"load-{x}"; Position=p x 0;Phase=0.0;Rate=1.0 }
    let outputs = [demand 160;demand 400]
    let inherited = DrivePlanning.route outputs Map.empty (p 40 0)
    let relocated = DrivePlanning.route outputs Map.empty (p 280 0)
    require (inherited.Nodes.Length=7 && relocated.Nodes.Length=5) "Relocation must prune now-unneeded input shafts."
    require (relocated.Edges.Length=4) "Fresh active route must contain exactly four meshes."
    require (DrivePlanning.gearUses relocated |> List.forall (fun (_,uses) -> not uses.IsEmpty)) "Every gear must be consumed."
    let raised = DrivePlanning.route outputs Map.empty (p 280 -60)
    let motor = raised.Nodes |> List.find (fun node -> node.Id=raised.Root)
    require (motor.Rate = -1.0) "An extra idler must invert the motor command, not reverse walking."
    require (raised.Nodes.Length=6 && raised.Edges.Length=5) "Raised motor needs an explicitly constructed extra mesh."
    let shortened = DrivePlanning.route [demand 160] Map.empty (p 40 0)
    require (shortened.Nodes.Length=3) "Unused output tail must disappear when its walking demand disappears."
    let direct = DrivePlanning.route [demand 160] Map.empty (p 160 0)
    require (direct.Nodes.Length=1 && direct.Edges.IsEmpty) "Direct drive must not retain a decorative gear."
    let fails action =
        try action (); false with :? ArgumentException -> true
    require (fails (fun () -> DrivePlanning.route [demand 160;{demand 400 with Rate=0.2}] Map.empty (p 40 0) |> ignore))
            "Unsupported reduction must be explicit."
    require (fails (fun () -> DrivePlanning.route [demand 160;{demand 400 with Phase=0.5}] Map.empty (p 40 0) |> ignore))
            "Wrong output phase must not be accepted."
    require (fails (fun () -> DrivePlanning.route outputs Map.empty (p 250 0) |> ignore)) "Off-lattice route must fail."
    printfn "Transmission self-tests passed: pruning, complete gear usage, direct drive, polarity, phase and unsupported routes."

let experiment (parent: string) (properties: string) (output: string) =
    let parent,output = Path.GetFullPath parent,Path.GetFullPath output
    if parent=output || output.StartsWith(parent+string Path.DirectorySeparatorChar,StringComparison.Ordinal)
       || parent.StartsWith(output+string Path.DirectorySeparatorChar,StringComparison.Ordinal) then
        invalidArg "output" "Output must be separate from the immutable parent."
    if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
        invalidArg "output" "Output must be empty."
    let digest path = File.ReadAllBytes path |> SHA256.HashData |> Convert.ToHexString |> _.ToLowerInvariant()
    use input = JsonDocument.Parse(File.ReadAllText properties)
    let meta = input.RootElement
    if meta.GetProperty("schemaVersion").GetInt32() <> 1 then invalidArg "mass-properties" "Expected schema 1."
    for file,key in ["program.json","programSha256";"realization.json","realizationSha256";"manifest.json","parentManifestSha256"] do
        if digest (Path.Combine(parent,file)) <> meta.GetProperty(key).GetString() then
            invalidArg "mass-properties" $"Input hash mismatch: {file}."
    let vector (element: JsonElement) =
        let p = Vec3.create (element.GetProperty("x").GetDouble()) (element.GetProperty("y").GetDouble())
                            (element.GetProperty("z").GetDouble())
        if not (Vec3.finite p) then invalidArg "mass-properties" "Non-finite geometry."
        p
    let massParts = meta.GetProperty("parts").EnumerateArray() |> Seq.toList
    let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
    let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let genome = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
    let screened = Realization.screen program 72 |> unwrap
    let baseline = WalkerRealization.compile screened genome |> unwrap
    let frames,coverage = WalkerClearance.inspect screened baseline.Genome
    let shaftIds = program.Motor.Body::(program.Transmissions |> List.collect (fun t -> [t.Input;t.Output])) |> Set.ofList
    let bodyById = program.Bodies |> List.map (fun b -> b.Id,b) |> Map.ofList
    let moving = program.Bodies |> List.filter (fun b -> not b.Grounded && not (shaftIds.Contains b.Id))
    let grounded = program.Bodies |> List.find _.Grounded
    let loads =
        program.Hinges |> List.collect (fun h ->
            [h.A.Body,h.B.Body;h.B.Body,h.A.Body] |> List.choose (fun (a,b) ->
                if shaftIds.Contains a && not (shaftIds.Contains b) && not bodyById[b].Grounded then Some a else None))
        |> Set.ofList |> Set.union (program.Feet |> List.map _.Port.Body |> List.filter shaftIds.Contains |> Set.ofList)
    let rec rates states = function
        | [] -> states
        | current::rest ->
            let fresh = program.Transmissions |> List.choose (fun edge ->
                if edge.Input=current && not (Map.containsKey edge.Output states) then Some (edge.Output,edge.Ratio)
                elif edge.Output=current && not (Map.containsKey edge.Input states) then Some (edge.Input,1.0/edge.Ratio)
                else None)
            let states = fresh |> List.fold (fun acc (id,ratio) -> Map.add id (states[current]*ratio) acc) states
            rates states (rest @ (fresh |> List.map fst))
    let rateById = rates (Map.ofList [(program.Motor.Body,1.0)]) [program.Motor.Body]
    if rateById.Count <> shaftIds.Count || program.Transmissions.Length <> shaftIds.Count-1 then
        invalidArg "parent" "Prototype requires a connected acyclic prescribed transmission."
    let demands = loads |> Set.toList |> List.map (fun id ->
        let pose = frames.Head.Poses[id]
        if frames |> List.exists (fun frame -> Vec3.distance frame.Poses[id].Origin pose.Origin > 1e-5) then
            invalidArg "parent" $"Drive output {id} does not have a fixed axis."
        { Id=id;Position={pose.Origin with Z=0.0};Phase=pose.Angle;Rate=rateById[id] })
    let demands = demands |> List.sortBy (fun node -> node.Position.X,node.Id)
    if demands.IsEmpty then invalidArg "parent" "No walking drive demands found."
    let existing = shaftIds |> Set.toList |> List.map (fun id -> bodyById[id].Initial.Origin,id) |> Map.ofList
    let sourceParts = baseline.Assembly.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
    let originalGears = baseline.Assembly.Parts |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome)
    let gearPair (edge: MotionTransmission) = [
        for a in originalGears |> List.filter (fun p -> p.Body=Some edge.Input) do
            for b in originalGears |> List.filter (fun p -> p.Body=Some edge.Output) do
                let na,nb = float (Catalog.gearTeeth a.Part).Value,float (Catalog.gearTeeth b.Part).Value
                if abs (edge.Ratio+na/nb)<1e-7 && abs(a.Position.Z-b.Position.Z)<1e-7
                   && abs(Vec3.distance a.Position b.Position-(na+nb)*1.25)<1e-6 then yield a.Id,b.Id
    ]
    let inheritedPairs = program.Transmissions |> List.map (fun edge ->
        match gearPair edge with
        | [a,b] -> {| input=edge.Input;output=edge.Output;gearA=a;gearB=b |}
        | _ -> invalidArg "parent" "Inherited transmission mesh is not unambiguous.")
    let consumed = inheritedPairs |> List.collect (fun pair -> [pair.gearA;pair.gearB]) |> Set.ofList
    let inheritedOrphans = originalGears |> List.filter (fun gear -> not (consumed.Contains gear.Id)) |> List.map _.Id
    let oldMotor = sourceParts[genome.MotorPart]
    let motorMass = massParts |> List.find (fun p -> p.GetProperty("id").GetString()=oldMotor.Id)
    let motorLow,motorHigh = vector (motorMass.GetProperty("low")),vector (motorMass.GetProperty("high"))
    let motorMassKg = motorMass.GetProperty("massKg").GetDouble()
    if not (Double.IsFinite motorMassKg) || motorMassKg<=0.0 then invalidArg "mass-properties" "Invalid motor mass."
    let center = (demands.Head.Position.X+(List.last demands).Position.X)/2.0
    let center = demands.Head.Position.X+60.0*Math.Round((center-demands.Head.Position.X)/60.0)
    let y = demands.Head.Position.Y
    let roots =
        [bodyById[program.Motor.Body].Initial.Origin
         demands.Head.Position;Vec3.create center y 0;(List.last demands).Position
         Vec3.create center (y-60.0) 0;Vec3.create center (y+60.0) 0] |> List.distinct
    let retained = (moving |> List.map _.Id |> Set.ofList) |> Set.union loads
    let physicalLoadParts =
        baseline.Genome.Bodies |> List.filter (fun body -> loads.Contains body.Body)
        |> List.collect (fun body ->
            body.Parts |> List.filter (fun p ->
                (Catalog.gearTeeth p.Part).IsNone && not (List.contains p.Part ["3713.dat";"4265c.dat"]))
            |> List.map (fun p -> body.Body,p))
    let cases = ResizeArray<_>()
    let rejected = ResizeArray<_>()
    Directory.CreateDirectory output |> ignore
    for rootIndex,root in List.indexed roots do
        let route = DrivePlanning.route demands existing root
        let nodes = route.Nodes |> List.map (fun node -> node.Id,node) |> Map.ofList
        let motorNode = nodes[route.Root]
        let kept = route.Nodes |> List.map _.Id |> Set.ofList
        let shaftBodies = route.Nodes |> List.map (fun node ->
            let sites = if loads.Contains node.Id then bodyById[node.Id].Sites else [Motion.site "root" 0 0]
            { Id=node.Id;Grounded=false;Initial={Origin=node.Position;Angle=node.Phase};Sites=sites })
        if moving.Length+shaftBodies.Length+1 > 32 then invalidArg "route" "Constructed motion graph exceeds 32 bodies."
        let bearingSites = route.Nodes |> List.map (fun node -> Motion.site ("constructed-bearing-"+node.Id) node.Position.X node.Position.Y)
        let childProgram = {
            program with
                Bodies = (moving |> List.map (fun body -> {body with Initial=frames.Head.Poses[body.Id]}))
                         @ shaftBodies @ [{grounded with Sites=grounded.Sites @ bearingSites}]
                Hinges = (program.Hinges |> List.filter (fun h ->
                    let bearing = (shaftIds.Contains h.A.Body && bodyById[h.B.Body].Grounded)
                                  || (shaftIds.Contains h.B.Body && bodyById[h.A.Body].Grounded)
                    not bearing && (not (shaftIds.Contains h.A.Body) || loads.Contains h.A.Body)
                                && (not (shaftIds.Contains h.B.Body) || loads.Contains h.B.Body)))
                         @ (route.Nodes |> List.map (fun node ->
                             Motion.couple ("constructed-bearing-"+node.Id)
                                 (Motion.port grounded.Id ("constructed-bearing-"+node.Id)) (Motion.port node.Id "root")))
                Motor = { Body=route.Root;Phase=motorNode.Phase }
                Transmissions = route.Edges |> List.map (fun edge -> Motion.transmit edge.Input edge.Output -1.0 edge.Phase)
        }
        let graph = Motion.compile childProgram |> Result.defaultWith (String.concat "; " >> invalidOp)
        let childFrames = Motion.sample graph (frames |> List.map (fun frame -> motorNode.Rate*frame.InputAngle))
                          |> Result.defaultWith invalidOp
        let positionErrors,angleErrors,footErrors =
            List.zip frames childFrames |> List.map (fun (before,after) ->
                let positions = retained |> Set.toList |> List.map (fun id ->
                    Vec3.distance before.Poses[id].Origin after.Poses[id].Origin)
                let angles = retained |> Set.toList |> List.map (fun id ->
                    DrivePlanning.angleError before.Poses[id].Angle after.Poses[id].Angle)
                let feet = program.Feet |> List.map (fun foot ->
                    let a = Motion.position screened.Graph before foot.Port |> Result.defaultWith invalidOp
                    let b = Motion.position graph after foot.Port |> Result.defaultWith invalidOp
                    Vec3.distance a b)
                List.max positions,List.max angles,List.max feet) |> List.unzip3
        let comparison = {
            SampledPhases=frames.Length;MaxBodyPositionErrorLdu=List.max positionErrors
            MaxBodyAngleErrorRadians=List.max angleErrors;MaxFootPositionErrorLdu=List.max footErrors
            MotorClockScale=motorNode.Rate;ReferenceMotorSpeed=genome.Speed
            ConstructedMotorSpeed=genome.Speed*motorNode.Rate }
        if comparison.MaxBodyPositionErrorLdu>1e-5 || comparison.MaxBodyAngleErrorRadians>1e-6
           || comparison.MaxFootPositionErrorLdu>1e-5 then
            invalidOp $"Motor root {root} changes the prescribed walking motion."
        let trajectory = List.zip frames childFrames |> List.map (fun (before,after) ->
            { ReferenceInputAngle=before.InputAngle;MotorInputAngle=after.InputAngle
              GearAngles=route.Nodes |> List.map (fun node -> node.Id,after.Poses[node.Id].Angle)
              Feet=program.Feet |> List.map (fun foot ->
                  let p = Motion.position graph after foot.Port |> Result.defaultWith invalidOp
                  let depth = genome.Feet |> List.find (fun f -> f.Foot=foot.Id) |> _.Depth
                  {p with Z=depth}) })
        for style,z,plane,face in ["outer-rear",150.0,60.0,1.0;"outer-front",-30.0,60.0,-1.0;
                                  "interior-forward-output",0.0,-40.0,1.0;"interior-rearward-output",120.0,160.0,-1.0] do
            let id = $"root-{rootIndex:D2}-{style}"
            let motorPosition = {root with Z=z}
            let motorRotation = {X=Vec3.create face 0 0;Y=Vec3.create 0 1 0;Z=Vec3.create 0 0 face}
            let socketDepths (node: DriveNode) =
                physicalLoadParts |> List.filter (fun (body,_) -> body=node.Id)
                |> List.collect (fun (_,part) -> (Catalog.get part.Part).Ports |> List.choose (fun port ->
                    let position = Vec3.add part.Position (Rotation.apply part.Rotation port.Position)
                    let axis = Rotation.apply part.Rotation port.Axis
                    if port.Kind=AxleHole && abs axis.Z>0.999
                       && abs(position.X-node.Position.X)<1e-5 && abs(position.Y-node.Position.Y)<1e-5 then Some position.Z else None))
            let rootDepths = socketDepths motorNode
            let throughHousing =
                rootDepths |> List.exists (fun depth -> (depth-z)*face > 20.000001)
            if throughHousing then
                rejected.Add {| id=id;code="load-shaft-would-pass-through-motor-housing"
                                motorPosition=motorPosition;loadSocketDepths=rootDepths |}
            else
                let uses = DrivePlanning.gearUses route |> Map.ofList
                let gears = route.Nodes |> List.choose (fun node ->
                    if uses[node.Id].IsEmpty then None
                    else Some { Teeth=24;Meshes=uses[node.Id]
                                Placement={Id="constructed-gear-"+node.Id;Part="3648b.dat";Color=4;Body=Some node.Id
                                           Position={node.Position with Z=plane};Rotation=Rotation.aroundZ node.Phase} })
                let shafts = route.Nodes |> List.map (fun node ->
                    let motorSockets = if node.Id=route.Root then [z;z+20.0*face] else []
                    let depths = [plane;plane-20.0;plane+20.0] @ socketDepths node @ motorSockets
                    let low,high = List.min depths,List.max depths
                    let part = ["3705.dat";"44294.dat";"3707.dat";"3708.dat"] |> List.tryFind (fun part ->
                        let a,b = Catalog.shaftRange part
                        b-a>=high-low-1e-6) |> Option.defaultWith (fun () -> invalidOp "Required axle span exceeds the supported stock.")
                    let a,b = Catalog.shaftRange part
                    let center =
                        if node.Id=route.Root && face>0.0 then high-b
                        elif node.Id=route.Root then low-a
                        else (low+high-a-b)/2.0
                    { Placement={Id="constructed-axle-"+node.Id;Part=part;Color=8;Body=None
                                 Position={node.Position with Z=center};Rotation=Rotation.shaft node.Phase}
                      SocketCentersZ=depths |> List.distinct |> List.sort
                      BearingTargets=[{node.Position with Z=plane-20.0};{node.Position with Z=plane+20.0}]
                      MinimumReachZ=center+a;MaximumReachZ=center+b })
                let motor = {oldMotor with Position=motorPosition;Rotation=motorRotation}
                let parts = (gears |> List.map _.Placement) @ (shafts |> List.map _.Placement) @ [motor]
                let meshes = route.Edges |> List.map (fun edge ->
                    let a,b = nodes[edge.Input],nodes[edge.Output]
                    if abs(Vec3.distance a.Position b.Position-60.0)>1e-6 then invalidOp "Wrong gear spacing."
                    let direction = atan2 (b.Position.Y-a.Position.Y) (b.Position.X-a.Position.X)
                    let toothError = DrivePlanning.angleError (24.0*(a.Phase+b.Phase)-48.0*direction) Math.PI
                    if toothError>1e-5 then invalidOp "Gear teeth do not match the required initial phase."
                    {| id=edge.Input+"->"+edge.Output;gearA="constructed-gear-"+edge.Input
                       gearB="constructed-gear-"+edge.Output;ratio = -1.0;planeZ=plane
                       centerSpacingLdu=60.0;toothPhaseResidualRadians=toothError |})
                for gear in gears do
                    if gear.Meshes.IsEmpty then invalidOp "Unconsumed constructed gear."
                for shaft in shafts do
                    if shaft.SocketCentersZ |> List.exists (fun z -> z<shaft.MinimumReachZ-1e-6 || z>shaft.MaximumReachZ+1e-6) then
                        invalidOp "Axle fails its required nominal reach."
                let corners = [for x in [motorLow.X;motorHigh.X] do
                                   for y in [motorLow.Y;motorHigh.Y] do
                                       for z in [motorLow.Z;motorHigh.Z] ->
                                           Vec3.add motorPosition (Rotation.apply motorRotation (Vec3.create x y z))]
                let low = Vec3.create (corners |> List.minBy _.X).X (corners |> List.minBy _.Y).Y (corners |> List.minBy _.Z).Z
                let high = Vec3.create (corners |> List.maxBy _.X).X (corners |> List.maxBy _.Y).Y (corners |> List.maxBy _.Z).Z
                let mountTargets = (Catalog.get motor.Part).Ports |> List.filter (fun p -> p.Kind=RoundHole)
                                   |> List.map (fun p -> {| position=Vec3.add motorPosition (Rotation.apply motorRotation p.Position);
                                                          axis=Rotation.apply motorRotation p.Axis |})
                let directory = Path.Combine(output,id)
                Directory.CreateDirectory directory |> ignore
                File.WriteAllText(Path.Combine(directory,"drive-program.json"),MotionExport.programJson childProgram)
                File.WriteAllText(Path.Combine(directory,"transmission.ldr"),
                    "0 OFFLINE TRANSMISSION SUBASSEMBLY: chassis, mounts and load connections are not constructed.\r\n"
                    + Export.ldraw {baseline.Assembly with Parts=parts})
                let layout = {|
                    id=id;status="constructed-drive-layout-requiring-chassis";root=route.Root;route=route
                    motor = {| placement=motor;massKg=motorMassKg;low=low;high=high
                               withinInheritedRailDepthSpan=low.Z>=0.0 && high.Z<=120.0
                               mountTargets=mountTargets;speed=genome.Speed*motorNode.Rate;maxTorque=genome.MaxTorque |}
                    gears=gears;meshes=meshes;shafts=shafts;comparison=comparison;frames=trajectory
                    removedSourceShaftBodies=shaftIds |> Set.filter (fun id -> not (kept.Contains id)) |> Set.toList
                    discardedUnneededSourceGears=originalGears |> List.filter (fun gear -> gear.Body |> Option.exists (fun body -> not (kept.Contains body))) |> List.map _.Id
                    replacementPolicy="Every gear and axle in this drive layout is freshly constructed; no inherited transmission part is copied."
                    unconstructed=["Chassis bearings and motor mounting pins/bracing are requests, not assembled parts."
                                   "Source walking crank sockets must be connected to these output axles."
                                   "Axle reach checks socket centers only; full wall engagement, retention and spacers remain unverified."
                                   "Housing check prevents obvious load shafts passing through the motor; other collisions are untested."
                                   "New transmission/chassis mass requires a fresh balance analysis; earlier packaging COM estimates cannot be reused."]
                |}
                File.WriteAllText(Path.Combine(directory,"transmission.json"),Realization.json layout+"\n")
                cases.Add layout
    File.Copy(Path.Combine(parent,"program.json"),Path.Combine(output,"walking-reference.json"))
    let report = {|
        schemaVersion=1;version="offline-demand-driven-transmission-v1";status="offline-constructed-drive-layouts"
        parent=parent;programSha256=digest (Path.Combine(parent,"program.json"))
        realizationSha256=digest (Path.Combine(parent,"realization.json"));massInputSha256=digest properties
        demands=demands;preservedWalkingBodies=retained |> Set.toList;motionCoverage=coverage
        inherited = {| gearCount=originalGears.Length;meshes=inheritedPairs;orphanGearIds=inheritedOrphans
                       motor=oldMotor;gearPositions=originalGears |> List.map _.Position |}
        layouts=cases |> Seq.toList;rejected=rejected |> Seq.toList
        counts = {| motorAxisChoices=roots.Length;layoutChoices=cases.Count;rejectedHousingChoices=rejected.Count
                    sampledReferencePhases=frames.Length;physicsTrials=0;generatedWholeWalkers=0 |}
        limitations=["Prototype supports a straight 60 LDU output lattice, 24T external spur gears, and parallel transverse axles."
                     "Required output rates must have magnitude one and phases compatible with this mesh lattice; unsupported demands fail explicitly."
                     "Motor command polarity follows the rebuilt route, preserving walking against the original reference clock."
                     "Output and retained walking-body motion match sampled reference poses; this is not dynamic qualification or a continuous proof."
                     "Gear usage, spacing, ratio, initial tooth phase and nominal axle reach are checked; actual collision geometry is not."
                     "Only standalone transmission LDraw layouts are exported, never admissible complete-walker candidates."
                     "No mounts, chassis synthesis, load-dependent deflection, friction, torque distribution, cables or balance requalification."
                     "No live evolution, scientific version, historical archive or authorization is modified."]
    |}
    let json = Realization.json report
    File.WriteAllText(Path.Combine(output,"transmission-plan.json"),json+"\n")
    let template = File.ReadAllText(Path.Combine(__SOURCE_DIRECTORY__,"transmission_preview.html"))
    File.WriteAllText(Path.Combine(output,"index.html"),template.Replace("__DRIVE_DATA__",json))
    printfn "%s" (Realization.json report.counts)

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--self-test"] -> selfTest ()
| ["--parent";parent;"--mass-properties";properties;"--output";output] -> experiment parent properties output
| _ -> invalidArg "arguments" "Expected --parent DIR --mass-properties JSON --output EMPTY_DIR, or --self-test."
