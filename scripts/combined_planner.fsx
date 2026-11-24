#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"
#load "PackagingSupport.fsx"

open System
open System.IO
open System.Collections.Generic
open System.Security.Cryptography
open System.Text.Json
open LegoWalkers
open PackagingSupport

type Envelope = {
    Id: string; Body: string; Kind: string; Low: Vec3; High: Vec3
    SegmentStart: Vec3; SegmentEnd: Vec3; Radius: float
}
type DriveInput = {
    Id: string; Root: string; ClockScale: float; MotorSpeed: float; MaxTorque: float
    DriveProgramPath: string; DriveProgramSha256: string; LdrawPath: string; LdrawSha256: string
    Parts: MassPart list; Placements: WalkerPlacement list; Envelopes: Envelope list
    BearingTargets: Vec3 list
}
type CombinedInput = {
    SchemaVersion: int; Parent: string; SourceMass: MassInput; SourceMassPath: string; SourceMassSha256: string
    DrivePlanPath: string; DrivePlanSha256: string; FreshDependencySha256: string
    RemovedLegacyIds: string list; LegacyEnvelopes: Envelope list; LegacyPlacements: WalkerPlacement list
    Drives: DriveInput list
}
type Clearance = {
    PartA: string; PartB: string; Family: string; ClearanceLdu: float
    InputAngleRadians: float; FirstInputAngleRadians: float; Static: bool
}
type Rod = { Id: string; Body: string; A: Vec3; B: Vec3 }
type Metrics = {
    WorstMarginLdu: float; MeanHeightLdu: float; OutsideSupportPhases: int; ClearanceScore: float
}
type Candidate = { Id: string; DriveId: string; BatteryIndex: int; ReceiverIndex: int; Metrics: Metrics }

module CombinedMath =
    let score contacts = contacts |> List.sumBy (fun c -> pown (WalkerClearance.warningMargin-c.ClearanceLdu) 2)
    let envelopeGap (e: Envelope) low high =
        match e.Kind with
        | "gear" -> PackagingMath.boxGap (e.Low,e.High) (low,high)
        | "shaft" ->
            let distance,_,_ = WalkerClearance.segmentBoxDistance e.SegmentStart e.SegmentEnd low high
            distance-e.Radius
        | _ -> invalidArg "envelope" $"Unsupported drive envelope: {e.Kind}"

    let hardwareDrive root motorId (envelopes: Envelope list) (target: Target) =
        envelopes |> List.choose (fun envelope ->
            let declaredInput = target.Id=motorId && envelope.Kind="shaft" && envelope.Body=root
            if declaredInput then None
            else
                let gap = envelopeGap envelope target.Low target.High
                if gap>WalkerClearance.warningMargin then None
                else Some {PartA=target.Id;PartB=envelope.Id;Family="static-hardware-drive";ClearanceLdu=gap
                           InputAngleRadians=0.0;FirstInputAngleRadians=0.0;Static=true})

    let driveMotion (envelopes: Envelope list) (angles: float array) (rods: Rod list array) =
        let contacts = Dictionary<string * string,Clearance>()
        for i in 0..angles.Length-1 do
            for envelope in envelopes do
                for rod in rods[i] do
                    if envelope.Body<>rod.Body then
                        let gap =
                            if envelope.Kind="gear" then
                                let distance,_,_ = WalkerClearance.segmentBoxDistance rod.A rod.B envelope.Low envelope.High
                                distance-WalkerClearance.rodRadius
                            else
                                let distance,_,_ = WalkerClearance.segmentDistance rod.A rod.B envelope.SegmentStart envelope.SegmentEnd
                                distance-WalkerClearance.rodRadius-envelope.Radius
                        if gap<=WalkerClearance.warningMargin then
                            let key = envelope.Id,rod.Id
                            let contact = {PartA=envelope.Id;PartB=rod.Id;Family="moving-drive-link";ClearanceLdu=gap
                                           InputAngleRadians=angles[i];FirstInputAngleRadians=angles[i];Static=false}
                            match contacts.TryGetValue key with
                            | true,previous when previous.ClearanceLdu>gap ->
                                contacts[key] <- {contact with FirstInputAngleRadians=previous.FirstInputAngleRadians}
                            | false,_ -> contacts.Add(key,contact)
                            | _ -> ()
        contacts.Values |> Seq.toList |> List.sortBy (fun c -> c.ClearanceLdu,c.PartA,c.PartB)

let selfTest () =
    let require condition message = if not condition then failwith message
    let p x y z = Vec3.create x y z
    require (abs (PackagingMath.meanOverAngles [|0.0;1.0;3.0|] [|0.0;1.0;3.0|]-1.5)<1e-9)
            "Refined sampling must not bias mean height."
    require (abs(PackagingMath.boxGap (p 0 0 0,p 10 10 10) (p 13 14 0,p 20 20 10)-5.0)<1e-9)
            "Separated boxes require Euclidean corner gap."
    let shaft = {Id="axle";Body="root";Kind="shaft";Low=p -4 -4 -14;High=p 4 4 14
                 SegmentStart=p 0 0 -10;SegmentEnd=p 0 0 10;Radius=4.0}
    require (CombinedMath.envelopeGap shaft (p -1 -1 -1) (p 1 1 1)<0.0)
            "An interior payload must not hide an intersecting axle."
    let s,r =
        let program,g = WalkerSeeds.fourBar 2
        let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
        let s = Realization.screen program 72 |> unwrap
        s,WalkerRealization.compile s g |> unwrap
    let _,coverage = WalkerClearance.inspect s r.Genome
    let target id =
        {Id=id;Part="58119.dat";MassKg=0.197;Position=Vec3.zero;Rotation=Rotation.identity
         Center=Vec3.zero;Low=p -1 -1 -1;High=p 1 1 1;ToleranceLdu=p 10 6 10
         InsideInheritedRailPlanes=true;MovingClearance=coverage}
    require ((CombinedMath.hardwareDrive "root" "motor" [shaft] (target "motor")).IsEmpty)
            "Only declared motor/input-axle engagement is exempt."
    require ((CombinedMath.hardwareDrive "root" "motor" [shaft] (target "payload-battery")).Length=1)
            "The same axle must still warn for a battery."
    let rod body = {Id="leg";Body=body;A=p -10 0 0;B=p 10 0 0}
    require ((CombinedMath.driveMotion [shaft] [|0.0|] [|[rod "other"]|]).Length=1)
            "New shafts must be checked against moving links."
    require ((CombinedMath.driveMotion [shaft] [|0.0|] [|[rod "root"]|]).IsEmpty)
            "Rigid same-body drive engagement is not a moving pair."
    printfn "Combined self-tests passed: weighted phase mean, box gaps, shaft/payload interference and scoped engagement."

let experiment (inputPath: string) (output: string) =
    let output = Path.GetFullPath output
    if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
        invalidArg "output" "Combined output must be empty."
    let digest path = File.ReadAllBytes path |> SHA256.HashData |> Convert.ToHexString |> _.ToLowerInvariant()
    let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase,
        UnmappedMemberHandling=System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
        RespectRequiredConstructorParameters=true)
    let input = JsonSerializer.Deserialize<CombinedInput>(File.ReadAllText inputPath,options)
    if input.SchemaVersion<>1 || input.Drives.Length<>6 then invalidArg "input" "Expected six combined motor layouts."
    for path,expected in [input.SourceMassPath,input.SourceMassSha256;input.DrivePlanPath,input.DrivePlanSha256] do
        if digest path<>expected then invalidArg "input" $"Combined input changed: {path}"
    let sourceMass = JsonSerializer.Deserialize<MassInput>(File.ReadAllText input.SourceMassPath,options)
    if Realization.json sourceMass<>Realization.json input.SourceMass then
        invalidArg "input" "Embedded source mass properties differ from their immutable source."
    for file,expected in ["program.json",input.SourceMass.ProgramSha256;"realization.json",input.SourceMass.RealizationSha256;
                          "manifest.json",input.SourceMass.ParentManifestSha256] do
        if digest(Path.Combine(input.Parent,file))<>expected then invalidArg "input" $"Parent changed: {file}"
    let parent = Path.GetFullPath input.Parent
    if output=parent || output.StartsWith(parent+string Path.DirectorySeparatorChar,StringComparison.Ordinal)
       || parent.StartsWith(output+string Path.DirectorySeparatorChar,StringComparison.Ordinal) then
        invalidArg "output" "Combined output must stay outside the parent."
    let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
    let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let genome = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
    let screened = Realization.screen program 72 |> unwrap
    let baseline = WalkerRealization.compile screened genome |> unwrap
    let frameList,coverage = WalkerClearance.inspect screened baseline.Genome
    let frames = List.toArray frameList
    let samples = PackagingMath.motionSamples screened baseline frameList 8.0 |> List.toArray
    let angles = frames |> Array.map _.InputAngle
    let massById = input.SourceMass.Parts |> List.map (fun item -> item.Id,item) |> Map.ofList
    let partsById = baseline.Assembly.Parts |> List.map (fun part -> part.Id,part) |> Map.ofList
    if massById.Count<>partsById.Count then invalidArg "input" "Source mass must cover every part."
    for item in input.SourceMass.Parts @ (input.Drives |> List.collect _.Parts) do
        if not (Double.IsFinite item.MassKg) || item.MassKg<=0.0
           || not (Vec3.finite item.Center && Vec3.finite item.Low && Vec3.finite item.High) then
            invalidArg "input" $"Invalid physical mass properties: {item.Id}"
    let removed = input.RemovedLegacyIds |> Set.ofList
    if removed |> Set.exists (partsById.ContainsKey >> not) then invalidArg "input" "Unknown obsolete transmission part."
    let electronics = Set.ofList ["payload-battery";"payload-receiver";genome.MotorPart]
    let held = input.SourceMass.Parts |> List.filter (fun item -> not (removed.Contains item.Id || electronics.Contains item.Id))
    let heldMass = held |> List.sumBy _.MassKg
    let heldMoments = frames |> Array.map (fun frame ->
        held |> List.fold (fun acc item ->
            Vec3.add acc (Vec3.scale item.MassKg (PackagingMath.world screened frame item.Body item.Center))) Vec3.zero)
    let originalMass = input.SourceMass.Parts |> List.sumBy _.MassKg
    let originalMoments = frames |> Array.map (fun frame ->
        input.SourceMass.Parts |> List.fold (fun acc item ->
            Vec3.add acc (Vec3.scale item.MassKg (PackagingMath.world screened frame item.Body item.Center))) Vec3.zero)
    let rods = frames |> Array.map (fun frame ->
        baseline.Genome.Bodies |> List.filter (fun body -> body.Body<>baseline.Assembly.Chassis)
        |> List.collect (fun body -> body.Parts |> List.collect (fun part ->
            let points = (Catalog.get part.Part).Ports |> List.map _.Position |> List.distinct
            List.pairwise points |> List.map (fun (a,b) ->
                let transform local = Vec3.add part.Position (Rotation.apply part.Rotation local)
                                      |> PackagingMath.world screened frame body.Body
                {Id=part.Id;Body=body.Body;A=transform a;B=transform b}))))
    let makeTarget (part: WalkerPlacement) (mass: MassPart) =
        let low,high = PackagingMath.bounds part.Position part.Rotation mass.Low mass.High
        {Id=part.Id;Part=part.Part;MassKg=mass.MassKg;Position=part.Position;Rotation=part.Rotation
         Center=mass.Center;Low=low;High=high;ToleranceLdu=Vec3.create 10 6 10
         InsideInheritedRailPlanes=low.Z>=0.0 && high.Z<=120.0
         MovingClearance=PackagingMath.hardwareClearance screened frameList baseline.Genome coverage part}
    let sourceTarget id = makeTarget {Id=id;Part=partsById[id].Part;Color=partsById[id].Color;
                                     Position=partsById[id].Position;Rotation=partsById[id].Rotation} massById[id]
    let originalTargets = [sourceTarget "payload-battery";sourceTarget "payload-receiver";sourceTarget genome.MotorPart]
    let sourcePlacement id =
        let p = partsById[id]
        {Id=p.Id;Part=p.Part;Color=p.Color;Position=p.Position;Rotation=p.Rotation}
    let centerX = samples |> Array.averageBy (fun sample -> sample.Feet |> List.averageBy _.Position.X)
                  |> fun value -> 20.0*Math.Round(value/20.0)
    let placements id xs ys = [
        for dx in xs do
            for y in ys do
                for z in [20.0;60.0;100.0] do
                    let position = Vec3.create (centerX+dx) y z
                    let part = {sourcePlacement id with Position=position}
                    let source = massById[id]
                    let mass = {source with Center=Vec3.add source.Center (Vec3.subtract position partsById[id].Position)}
                    yield makeTarget part mass
    ]
    let batteries = placements "payload-battery" [-160.0;-80.0;0.0;80.0;160.0] [-192.0;-168.0;-144.0;-120.0;-96.0;-72.0;-48.0;-24.0;0.0] |> List.toArray
    let receivers = placements "payload-receiver" [-160.0;-80.0;0.0;80.0;160.0] [-192.0;-144.0;-96.0;-48.0;0.0] |> List.toArray
    let halfPlanes = samples |> Array.map (fun sample ->
        (sample.SupportHull @ [sample.SupportHull.Head]) |> List.pairwise
        |> List.map (fun (a,b) ->
            let length = sqrt(pown (b.X-a.X) 2+pown (b.Z-a.Z) 2)
            -(b.Z-a.Z)/length,(b.X-a.X)/length,((b.Z-a.Z)*a.X-(b.X-a.X)*a.Z)/length) |> List.toArray)
    let balances total (moments: Vec3 array) (targets: Target list) =
        let weighted = targets |> List.fold (fun acc target -> Vec3.add acc (Vec3.scale target.MassKg target.Center)) Vec3.zero
        moments |> Array.mapi (fun i moment ->
            let center = Vec3.scale (1.0/total) (Vec3.add moment weighted)
            let margin = halfPlanes[i] |> Array.fold (fun gap (x,z,offset) -> min gap (x*center.X+z*center.Z+offset)) Double.PositiveInfinity
            {Center=center;MarginLdu=margin;HeightLdu=samples[i].GroundY-center.Y})
    let metrics total moments targets clearanceScore =
        let phases = balances total moments targets
        {WorstMarginLdu=phases |> Array.map _.MarginLdu |> Array.min
         MeanHeightLdu=PackagingMath.meanOverAngles angles (phases |> Array.map _.HeightLdu)
         OutsideSupportPhases=phases |> Array.filter (fun phase -> phase.MarginLdu<0.0) |> Array.length
         ClearanceScore=clearanceScore}
    let targetContacts (target: Target) =
        target.MovingClearance.Contacts |> List.map (fun c ->
            {PartA=c.Parts[0];PartB=c.Parts[1];Family="moving-hardware-link";ClearanceLdu=c.ClearanceLdu
             InputAngleRadians=c.InputAngleRadians;FirstInputAngleRadians=c.FirstInputAngleRadians;Static=false})
    let originalDriveContacts = CombinedMath.driveMotion input.LegacyEnvelopes angles rods
    let originalHardwareDrive = originalTargets |> List.collect (CombinedMath.hardwareDrive program.Motor.Body genome.MotorPart input.LegacyEnvelopes)
    let originalScore = (originalTargets |> List.sumBy _.MovingClearance.Score)
                        + CombinedMath.score (originalDriveContacts @ originalHardwareDrive)
    let referenceMetrics = metrics originalMass originalMoments [] originalScore
    let reference = {|id="inherited";driveId="inherited";targets=originalTargets;totalMassKg=originalMass;
                     metrics=referenceMetrics;phases=balances originalMass originalMoments [] |> Array.toList;
                     clearance=List.collect targetContacts originalTargets @ originalDriveContacts @ originalHardwareDrive |}
    let selected = ResizeArray<_>()
    let driveReports = ResizeArray<_>()
    let programCache = Dictionary<string,MotionFrame array>()
    let mutable combinations,compatibleCount,housingExcluded,groundExcluded = 0,0,0,0
    let mutable maxMotionError = 0.0
    for drive in input.Drives do
        for path,expected in [drive.DriveProgramPath,drive.DriveProgramSha256;drive.LdrawPath,drive.LdrawSha256] do
            if digest path<>expected then invalidArg "input" $"Drive changed: {path}"
        if abs(abs drive.ClockScale-1.0)>1e-9 || drive.MotorSpeed<>genome.Speed*drive.ClockScale then
            invalidArg "input" "Motor command does not preserve the reference clock."
        let childProgram = MotionExport.readProgramDocument drive.DriveProgramPath
        if childProgram.Feet<>program.Feet then invalidArg "input" "Drive changed walking outputs."
        let childFrames =
            match programCache.TryGetValue drive.DriveProgramSha256 with
            | true,cached -> cached
            | _ ->
                let graph = Motion.compile childProgram |> Result.defaultWith (String.concat "; " >> invalidOp)
                let solved = Motion.sample graph (angles |> Array.map ((*) drive.ClockScale) |> Array.toList)
                             |> Result.defaultWith invalidOp |> List.toArray
                let shaftIds = program.Motor.Body::(program.Transmissions |> List.collect (fun edge -> [edge.Input;edge.Output])) |> Set.ofList
                let required = program.Bodies |> List.filter (fun body ->
                    not body.Grounded && (not (shaftIds.Contains body.Id)
                    || (childProgram.Bodies |> List.exists (fun child -> child.Id=body.Id))))
                for i in 0..frames.Length-1 do
                    for body in required do
                        let a,b = frames[i].Poses[body.Id],solved[i].Poses[body.Id]
                        let error = Vec3.distance a.Origin b.Origin
                        maxMotionError <- max maxMotionError error
                        if error>1e-5 || abs(atan2 (sin(a.Angle-b.Angle)) (cos(a.Angle-b.Angle)))>1e-6 then
                            invalidArg "input" "Constructed drive changes a retained body's motion."
                programCache.Add(drive.DriveProgramSha256,solved)
                solved
        let mass = drive.Parts |> List.map (fun part -> part.Id,part) |> Map.ofList
        if mass.Count<>drive.Placements.Length then invalidArg "input" "Drive part mass ledger is incomplete."
        for part in drive.Placements do
            if not (mass.ContainsKey part.Id) || mass[part.Id].Part<>part.Part || not (Rotation.valid part.Rotation && Vec3.finite part.Position) then
                invalidArg "input" "Invalid drive placement/mass correspondence."
        let motor = drive.Placements |> List.find (fun part -> part.Id=genome.MotorPart)
        let motorTarget = makeTarget motor mass[motor.Id]
        let driveParts = drive.Parts |> List.filter (fun item -> item.Id<>motor.Id)
        let total = heldMass+(drive.Parts |> List.sumBy _.MassKg)+massById["payload-battery"].MassKg+massById["payload-receiver"].MassKg
        let moments = childFrames |> Array.mapi (fun i frame ->
            driveParts |> List.fold (fun acc item ->
                let center = PackagingMath.movePoint childFrames[0].Poses[item.Body] frame.Poses[item.Body] item.Center
                Vec3.add acc (Vec3.scale item.MassKg center)) heldMoments[i])
        let movingDrive = CombinedMath.driveMotion drive.Envelopes angles rods
        let motorDrive = CombinedMath.hardwareDrive drive.Root genome.MotorPart drive.Envelopes motorTarget
        let fixedScore = CombinedMath.score (movingDrive @ motorDrive)+motorTarget.MovingClearance.Score
        let batteryDrive = batteries |> Array.map (CombinedMath.hardwareDrive drive.Root genome.MotorPart drive.Envelopes)
        let receiverDrive = receivers |> Array.map (CombinedMath.hardwareDrive drive.Root genome.MotorPart drive.Envelopes)
        let candidates = ResizeArray<Candidate>()
        let mutable driveHousing,driveGround = 0,0
        for i in 0..batteries.Length-1 do
            for j in 0..receivers.Length-1 do
                combinations <- combinations+1
                let battery,receiver = batteries[i],receivers[j]
                let targets = [battery;receiver;motorTarget]
                let overlaps =
                    PackagingMath.overlap (battery.Low,battery.High) (receiver.Low,receiver.High)
                    || PackagingMath.overlap (battery.Low,battery.High) (motorTarget.Low,motorTarget.High)
                    || PackagingMath.overlap (receiver.Low,receiver.High) (motorTarget.Low,motorTarget.High)
                let ground = targets |> List.exists (fun target -> samples |> Array.exists (fun sample -> target.High.Y>=sample.GroundY))
                if overlaps then housingExcluded<-housingExcluded+1;driveHousing<-driveHousing+1
                elif ground then groundExcluded<-groundExcluded+1;driveGround<-driveGround+1
                else
                    compatibleCount <- compatibleCount+1
                    let clearanceScore =
                        fixedScore + battery.MovingClearance.Score + receiver.MovingClearance.Score +
                        CombinedMath.score batteryDrive[i] + CombinedMath.score receiverDrive[j]
                    candidates.Add {Id = $"{drive.Id}-b{i:D3}-r{j:D3}";DriveId=drive.Id;BatteryIndex=i;ReceiverIndex=j
                                    Metrics=metrics total moments targets clearanceScore}
        let rankBalance (c: Candidate) = -c.Metrics.WorstMarginLdu,c.Metrics.MeanHeightLdu,c.Metrics.ClearanceScore,c.Id
        let rankClearance (c: Candidate) = c.Metrics.ClearanceScore,-c.Metrics.WorstMarginLdu,c.Metrics.MeanHeightLdu,c.Id
        let shortlisted =
            [(candidates |> Seq.sortBy rankBalance |> Seq.truncate 2 |> Seq.toList)
             (candidates |> Seq.sortBy rankClearance |> Seq.truncate 2 |> Seq.toList)]
            |> List.concat |> List.distinctBy _.Id
        for candidate in shortlisted do
            let targets = [batteries[candidate.BatteryIndex];receivers[candidate.ReceiverIndex];motorTarget]
            let contacts = (targets |> List.collect targetContacts) @ movingDrive @ motorDrive
                           @ batteryDrive[candidate.BatteryIndex] @ receiverDrive[candidate.ReceiverIndex]
                           |> List.sortBy (fun c -> c.ClearanceLdu,c.PartA,c.PartB)
            selected.Add {|id=candidate.Id;driveId=drive.Id;targets=targets;totalMassKg=total;metrics=candidate.Metrics;
                           phases=balances total moments targets |> Array.toList;clearance=contacts |}
        driveReports.Add {|id=drive.Id;root=drive.Root;motorSpeed=drive.MotorSpeed;maxTorque=drive.MaxTorque;
                           placements=drive.Placements;envelopes=drive.Envelopes;bearingTargets=drive.BearingTargets;
                           stockParts=drive.Parts;transmissionMassKg=driveParts |> List.sumBy _.MassKg;
                           totalMassKg=total;compatibleTargets=candidates.Count;housingExcluded=driveHousing;
                           groundExcluded=driveGround;fixedClearanceScore=fixedScore;
                           selectedTargets=shortlisted.Length;
                           gearAngles=childFrames |> Array.map (fun frame ->
                               drive.Envelopes |> List.filter (fun envelope -> envelope.Kind="gear")
                               |> List.map (fun envelope -> envelope.Id,frame.Poses[envelope.Body].Angle)) |}
    let report = {|
        schemaVersion=1;version="offline-combined-packaging-v1";status="advisory-combined-layouts"
        inputSha256=digest inputPath;programSha256=input.SourceMass.ProgramSha256
        realizationSha256=input.SourceMass.RealizationSha256;drivePlanSha256=input.DrivePlanSha256
        sourceMassSha256=input.SourceMassSha256;freshDependencySha256=input.FreshDependencySha256
        parent=parent;motionCoverage=coverage;motionFrames=samples;reference=reference
        drives=driveReports |> Seq.toList;plans=selected |> Seq.toList
        massLedger = {| heldPartIds=held |> List.map _.Id;heldMassKg=heldMass
                        removedLegacyIds=input.RemovedLegacyIds
                        removedLegacyMassKg=input.SourceMass.Parts |> List.filter (fun item -> removed.Contains item.Id) |> List.sumBy _.MassKg
                        sourceParts=input.SourceMass.Parts;sourceTotalMassKg=originalMass |}
        sourceDrive = {|placements=input.LegacyPlacements;envelopes=input.LegacyEnvelopes;root=program.Motor.Body
                        gearAngles=frames |> Array.map (fun frame ->
                            input.LegacyEnvelopes |> List.filter (fun envelope -> envelope.Kind="gear")
                            |> List.map (fun envelope -> envelope.Id,frame.Poses[envelope.Body].Angle)) |}
        placementGrid = {| batteries=batteries.Length;receivers=receivers.Length
                           batteryPositions=batteries |> Array.map _.Position;receiverPositions=receivers |> Array.map _.Position |}
        counts = {| combinedTargets=combinations;compatibleTargets=compatibleCount;housingExcluded=housingExcluded
                    groundExcluded=groundExcluded;motorLayouts=input.Drives.Length;selectedTargets=selected.Count
                    sampledPhases=frames.Length;uniqueDrivePrograms=programCache.Count;maxBodyPositionErrorLdu=maxMotionError
                    physicsTrials=0;generatedWholeWalkers=0 |}
        limitations = ["Offline mass-and-packaging targets and rebuilt transmission subassemblies, not constructed robots."
                       "Every new gear, axle and motor mass is estimated from its actual local LDraw mesh; payloads retain the existing battery/receiver profile."
                       "Old transmission gears, axles and bushings are removed from the mass ledger, then the rebuilt stock parts are added exactly once."
                       "Inherited chassis and old electronics supports/mounts retain their mass as an explicit common surrogate."
                       "New chassis bearings, bracing, retention spacers and mounts are unconstructed and have unknown additional mass."
                       "Stance assumes vertical settling to the lowest foot and an 8 LDU band; support is per-phase nominal contact disks, not contact forces."
                       "Mean COM height is integrated over reference phase using trapezoids; refined samples do not receive extra time weight."
                       "Electronics use the existing capsule/box motion checker; transmission gears use rotation-invariant mesh-bounded boxes and axles use mesh-bounded capsules."
                       "Transmission/leg checks use 3 LDU rods through catalogue sockets; sole volumes, one-socket parts and exact collision surfaces remain omitted."
                       "Same-body drive/link pairs and the exact motor/input-axle pair are exempt; other payload/shaft pairs are not."
                       "No old-frame or new-frame collision check, mesh-to-mesh dynamics, load deflection, cables, friction or stability qualification."
                       "All candidates share the sampled reference cycle; no candidate-specific adaptive refinement or between-sample safety proof."
                       "Housing/ground exclusions only shortlist this bounded offline search; remaining clearance warnings are advisory."
                       "Walking is rechecked against the original reference clock; no live evolution, version, archive or authorization changes."]
    |}
    Directory.CreateDirectory output |> ignore
    File.Copy(Path.Combine(parent,"program.json"),Path.Combine(output,"walking-reference.json"))
    for drive in input.Drives do
        let directory = Path.Combine(output,"drives",drive.Id)
        Directory.CreateDirectory directory |> ignore
        File.Copy(drive.LdrawPath,Path.Combine(directory,"transmission.ldr"))
        File.Copy(drive.DriveProgramPath,Path.Combine(directory,"drive-program.json"))
    for plan in selected do
        let drive = input.Drives |> List.find (fun drive -> drive.Id=plan.driveId)
        let additions = plan.targets |> List.filter (fun target -> target.Id<>genome.MotorPart)
                        |> List.map (fun target ->
                            let source = partsById[target.Id]
                            {source with Position=target.Position;Rotation=target.Rotation})
        let parts = drive.Placements |> List.map (fun p ->
            {Id=p.Id;Part=p.Part;Color=p.Color;Position=p.Position;Rotation=p.Rotation;Body=None})
        File.WriteAllText(Path.Combine(output,plan.id+".ldr"),
            "0 OFFLINE COMBINED PACKAGING TARGET: chassis, mounting and output connections are unconstructed.\r\n"
            + Export.ldraw {baseline.Assembly with Parts=parts @ additions})
    let json = Realization.json report
    File.WriteAllText(Path.Combine(output,"combined-plan.json"),json+"\n")
    let template = File.ReadAllText(Path.Combine(__SOURCE_DIRECTORY__,"combined_preview.html"))
    File.WriteAllText(Path.Combine(output,"index.html"),template.Replace("__COMBINED_DATA__",json))
    printfn "%s" (Realization.json report.counts)

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--self-test"] -> selfTest ()
| ["--input";input;"--output";output] -> experiment input output
| _ -> invalidArg "arguments" "Expected --input COMBINED_INPUT_JSON --output EMPTY_DIR, or --self-test."
