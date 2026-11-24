#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"
#load "PackagingSupport.fsx"

open System
open System.IO
open System.Security.Cryptography
open System.Text.Json
open LegoWalkers
open PackagingSupport

let selfTest () =
    let require condition message = if not condition then failwith message
    let p x z = Vec3.create x 0 z
    let square = PackagingMath.hull [p -10 -10;p 10 -10;p 10 10;p -10 10;p 0 0;p -10 -10]
    require (square.Length = 4) "Hull must remove interior and duplicate points."
    require (abs (PackagingMath.margin square (p 0 0)-10.0) < 1e-8) "Interior margin."
    require (abs (PackagingMath.margin square (p 20 0)+10.0) < 1e-8) "Exterior signed margin."
    require (abs (PackagingMath.margin [p -10 0;p 10 0] (p 0 20)+20.0) < 1e-8) "Degenerate support."
    let foot id y z = { Id = id; LogicalFoot = id; Position = Vec3.create 0 y z; Radius = 2.0 }
    let _,first,hull = PackagingMath.stance 4.0 [foot "near" 100 0;foot "far" 80 120]
    require (first = ["near"]) "Swinging feet must not enlarge the support polygon."
    require (PackagingMath.margin hull (p 0 60) < -50.0) "Swept-foot union must not masquerade as stance."
    let _,second,_ = PackagingMath.stance 4.0 [foot "near" 80 0;foot "far" 100 120]
    require (second = ["far"]) "Stance must change with phase and physical depth."
    require (PackagingMath.overlap (p -1 -1,p 1 1) (p 0 0,p 2 2) = false) "Zero-height boxes must not overlap."
    let lo,hi = PackagingMath.bounds (Vec3.create 10 20 30) Rotation.identity (Vec3.create -1 -2 -3) (Vec3.create 1 2 3)
    require (lo = Vec3.create 9 18 27 && hi = Vec3.create 11 22 33) "Housing bounds must translate."
    printfn "Packaging self-tests passed (hull, signed/degenerate margin, phase stance, physical depth, boxes)."

let experiment (parent: string) (properties: string) (output: string) =
    let parent,output = Path.GetFullPath parent,Path.GetFullPath output
    if parent = output || output.StartsWith(parent+string Path.DirectorySeparatorChar, StringComparison.Ordinal)
       || parent.StartsWith(output+string Path.DirectorySeparatorChar, StringComparison.Ordinal) then
        invalidArg "output" "Experiment output must be separate from its immutable parent."
    if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
        invalidArg "output" "Experiment output must be empty."
    let digest path = File.ReadAllBytes path |> SHA256.HashData |> Convert.ToHexString |> _.ToLowerInvariant()
    let options = JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                                       UnmappedMemberHandling = System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
                                       RespectRequiredConstructorParameters = true)
    let input = JsonSerializer.Deserialize<MassInput>(File.ReadAllText properties,options)
    if isNull (box input) || input.SchemaVersion <> 1 || input.Parts.IsEmpty then
        invalidArg "mass-properties" "Expected schema 1 and complete part mass properties."
    for file,expected in ["manifest.json",input.ParentManifestSha256;"program.json",input.ProgramSha256;
                         "realization.json",input.RealizationSha256] do
        if digest (Path.Combine(parent,file)) <> expected then
            invalidArg "mass-properties" $"Mass properties do not belong to the exact parent {file}."
    let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
    let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let genome = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
    let screened = Realization.screen program 72 |> unwrap
    let baseline = WalkerRealization.compile screened genome |> unwrap
    let allParts = baseline.Assembly.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
    if input.Parts.Length <> allParts.Count || (input.Parts |> List.map _.Id |> Set.ofList).Count <> allParts.Count then
        invalidArg "mass-properties" "Mass input must cover each assembly part exactly once."
    for item in input.Parts do
        match Map.tryFind item.Id allParts with
        | None -> invalidArg "mass-properties" $"Unknown part {item.Id}."
        | Some part ->
            if part.Part <> item.Part || not (Double.IsFinite item.MassKg) || item.MassKg <= 0.0
               || not (Vec3.finite item.Center && Vec3.finite item.Low && Vec3.finite item.High)
               || item.Low.X > item.High.X || item.Low.Y > item.High.Y || item.Low.Z > item.High.Z
               || not (screened.Frames.Head.Poses.ContainsKey item.Body)
               || (part.Body.IsSome && part.Body.Value <> item.Body) then
                invalidArg "mass-properties" $"Invalid mass, envelope or body ownership for {item.Id}."
    let frames,coverage = WalkerClearance.inspect screened baseline.Genome
    let stanceBand = 8.0
    let samples = PackagingMath.motionSamples screened baseline frames stanceBand
    let movable = Set.ofList ["payload-battery";"payload-receiver"]
    let fixedMass = input.Parts |> List.filter (fun item -> not (movable.Contains item.Id))
    let moments = frames |> List.map (fun frame ->
        fixedMass |> List.fold (fun total item ->
            let position = PackagingMath.world screened frame item.Body item.Center
            Vec3.add total (Vec3.scale item.MassKg position)) Vec3.zero)
    let totalMass = input.Parts |> List.sumBy _.MassKg
    if not (Double.IsFinite totalMass) then invalidArg "mass-properties" "Total mass must be finite."
    let sourceParts = baseline.Genome.Bodies |> List.collect _.Parts |> List.map (fun p -> p.Id,p) |> Map.ofList
    let mass id = input.Parts |> List.find (fun item -> item.Id = id)
    let chassis = baseline.Genome.Bodies |> List.find (fun b -> b.Body = baseline.Assembly.Chassis)
    let railPlanes =
        chassis.Parts |> List.filter (fun p -> p.Id.StartsWith("rail-") && p.Part = "3703.dat")
        |> List.collect (fun p -> (Catalog.get p.Part).Ports |> List.choose (fun port ->
            let axis = Rotation.apply p.Rotation port.Axis
            if port.Kind = RoundHole && abs axis.Z > 0.999 then
                Some (Vec3.add p.Position (Rotation.apply p.Rotation port.Position)).Z
            else None))
    if railPlanes.IsEmpty then invalidArg "parent" "No canonical transverse chassis rails for the interior reference."
    let nearPlane,farPlane = List.min railPlanes,List.max railPlanes
    let makeTarget id position =
        let source,item = sourceParts[id],mass id
        let low,high = PackagingMath.bounds position source.Rotation item.Low item.High
        let clearance = PackagingMath.hardwareClearance screened frames baseline.Genome coverage {source with Position=position}
        { Id = id; Part = item.Part; MassKg = item.MassKg; Position = position; Rotation = source.Rotation
          Center = Vec3.add item.Center (Vec3.subtract position source.Position)
          Low = low; High = high; ToleranceLdu = Vec3.create 10 6 10
          InsideInheritedRailPlanes = low.Z >= nearPlane && high.Z <= farPlane
          MovingClearance = clearance }
    let motor = sourceParts[baseline.Genome.MotorPart]
    let motorMass = mass motor.Id
    let motorBox = PackagingMath.bounds motor.Position motor.Rotation motorMass.Low motorMass.High
    let plan id (targets: Target list) =
        let weighted = targets |> List.fold (fun moment target ->
            Vec3.add moment (Vec3.scale target.MassKg target.Center)) Vec3.zero
        let phases = List.zip moments samples |> List.map (fun (moment,sample) ->
            let center = Vec3.scale (1.0/totalMass) (Vec3.add moment weighted)
            { Center = center; MarginLdu = PackagingMath.margin sample.SupportHull center
              HeightLdu = sample.GroundY-center.Y })
        let boxes = (motor.Id,motorBox)::(targets |> List.map (fun t -> t.Id,(t.Low,t.High)))
        let overlaps = [
            for i in 0..boxes.Length-1 do
                for j in i+1..boxes.Length-1 do
                    let a,boxA = boxes[i]
                    let b,boxB = boxes[j]
                    if PackagingMath.overlap boxA boxB then yield [a;b]
        ]
        { Id = id; Targets = targets; Phases = phases; WorstMarginLdu = phases |> List.map _.MarginLdu |> List.min
          MeanHeightLdu = phases |> List.averageBy _.HeightLdu
          OutsideSupportPhases = phases |> List.filter (fun p -> p.MarginLdu < 0.0) |> List.length
          MovingClearanceScore = targets |> List.sumBy _.MovingClearance.Score
          HousingOverlapPairs = overlaps
          GroundIntrusionPhases = samples |> List.filter (fun sample ->
              targets |> List.exists (fun target -> target.High.Y >= sample.GroundY)) |> List.length }
    let inherited = plan "inherited" [for id in ["payload-battery";"payload-receiver"] -> makeTarget id sourceParts[id].Position]
    let snap value = 20.0*Math.Round(value/20.0)
    let centerX = samples |> List.averageBy (fun sample -> sample.Feet |> List.averageBy _.Position.X) |> snap
    let centerZ = snap ((nearPlane+farPlane)/2.0)
    let candidates id xs ys = [
        for dx in xs do
            for y in ys do
                for dz in [-40.0;0.0;40.0] do
                    yield makeTarget id (Vec3.create (centerX+dx) y (centerZ+dz))
    ]
    let batteries = candidates "payload-battery" [-40.0;0.0;40.0] [-48.0;-24.0;0.0]
    let receivers = candidates "payload-receiver" [-160.0;-80.0;0.0;80.0;160.0] [-144.0;-96.0;-48.0]
    let proposals = [
        for i,battery in List.indexed batteries do
            for j,receiver in List.indexed receivers do
                yield plan $"target-{i:D2}-{j:D2}" [battery;receiver]
    ]
    let compatible = proposals |> List.filter (fun p -> p.HousingOverlapPairs.IsEmpty && p.GroundIntrusionPhases = 0)
    let rankBalance (p: Plan) = -p.WorstMarginLdu,p.MeanHeightLdu,p.MovingClearanceScore,p.Id
    let rankClearance (p: Plan) = p.MovingClearanceScore,-p.WorstMarginLdu,p.MeanHeightLdu,p.Id
    let selected =
        [compatible |> List.sortBy rankBalance |> List.truncate 2
         compatible |> List.sortBy rankClearance |> List.truncate 2
         compatible |> List.filter (fun p -> p.Targets |> List.forall _.InsideInheritedRailPlanes)
                    |> List.sortBy rankClearance |> List.truncate 2]
        |> List.concat |> List.distinctBy _.Id
    let bearings =
        baseline.Assembly.Joints |> List.filter (fun joint -> joint.BodyA = chassis.Body || joint.BodyB = chassis.Body)
        |> List.map (fun joint -> {| id = joint.Id; position = joint.Position; axis = joint.Axis |})
    let sockets (target: Target) =
        (Catalog.get target.Part).Ports |> List.filter (fun port -> port.Kind = RoundHole)
        |> List.map (fun port -> {| position = Vec3.add target.Position (Rotation.apply target.Rotation port.Position);
                                   axis = Rotation.apply target.Rotation port.Axis |})
    let requirements = selected |> List.map (fun p ->
        {| planId = p.Id
           mountRequests = p.Targets |> List.map (fun t -> {| componentId = t.Id; socketTargets = sockets t;
                                                             targetToleranceLdu = t.ToleranceLdu |})
           reservedComponentVolumes = p.Targets |> List.map (fun t -> {| id = t.Id; low = t.Low; high = t.High |})
           unmet = ["New frame, bracing and mounting recipes must be synthesized."
                    "Mount engagement, bearing support, shaft retention, stiffness and assembly order are unverified."
                    "Placement tolerances are requests; their balance and clearance envelopes have not been tested."] |})
    let report = {|
        schemaVersion = 1; version = "offline-mass-packaging-v1"; status = "placement-targets-only"
        parent = parent; parentGenotypeId = baseline.GenotypeId; mechanismId = screened.MechanismId
        programSha256 = input.ProgramSha256; realizationSha256 = input.RealizationSha256
        massInputSha256 = digest properties; payloadProfileSha256 = input.PayloadProfileSha256
        dependencySha256 = input.DependencySha256
        coreAssemblySha256 = digest (Path.Combine(__SOURCE_DIRECTORY__,"../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"))
        totalMassKg = totalMass; heldMassKg = fixedMass |> List.sumBy _.MassKg
        motor = {| id = motor.Id; massKg = motorMass.MassKg; position = motor.Position
                   low = fst motorBox; high = snd motorBox; anchorConstraint = "Existing motor/transmission anchor held fixed." |}
        inheritedRailPlanesZ = [nearPlane;farPlane]; bearingRequests = bearings
        motionCoverage = coverage; motionFrames = samples
        stance = {| source = "per-phase-lowest-foot-with-assumed-vertical-settling"; bandLdu = stanceBand
                    contactShape = "12-sided inscribed nominal contact disks; not measured load-bearing patches" |}
        inherited = inherited; plans = selected; constructionRequests = requirements
        counts = {| individualPlacementChecks = 2+batteries.Length+receivers.Length
                    combinedTargets = proposals.Length; housingCompatibleTargets = compatible.Length
                    housingOverlapTargets = proposals |> List.filter (fun p -> not p.HousingOverlapPairs.IsEmpty) |> List.length
                    groundIntrusionTargets = proposals |> List.filter (fun p -> p.GroundIntrusionPhases > 0) |> List.length
                    selectedTargets = selected.Length; sampledPhases = samples.Length
                    dynamicTrials = 0; generatedRobots = 0 |}
        ranking = "Two best conditional balance, two best moving clearance, and two best fully interior by clearance; deduplicated. Housing/ground conflicts excluded from this target shortlist, not a physical gate."
        limitations = ["No chassis or mount is constructed; no LDraw candidate or admission is generated."
                       "Exact internal motion and physical leg depths are retained; no phase or linkage mutation."
                       "Stance assumes vertical settling to the lowest foot at each phase, with an 8 LDU near-ground band."
                       "There is no solved world-space gait, contact force, friction, pitch, roll or dynamic stability proof."
                       "Component masses use the existing payload profile and ABS mesh estimates, including connector mass."
                       "Inherited frame, mounts and supports retain their mass and motion as a comparison surrogate; synthesized chassis mass is unknown."
                       "Interior means housing depth lies between the main inherited rail planes, not that the old frame can mount or contain it."
                       "Target clearance uses thin moving links; old chassis obstacles are deliberately removed, and new structure is not checked."
                       "Housing pair checks use conservative world AABBs. Cables, connectors, service access and motor internals are omitted."
                       "Baseline whole-model warnings remain in motionCoverage; packaging scores count only battery/receiver pairs."
                       "Incomplete cycle coverage invalidates whole-cycle conclusions; between-sample contacts remain possible."
                       "Negative support margins remain warnings: a less bad layout is not a stable walker."]
    |}
    Directory.CreateDirectory output |> ignore
    File.Copy(Path.Combine(parent,"program.json"),Path.Combine(output,"program.json"))
    File.Copy(Path.Combine(parent,"realization.json"),Path.Combine(output,"inherited-realization.json"))
    let json = Realization.json report
    File.WriteAllText(Path.Combine(output,"packaging-plan.json"),json+"\n")
    let template = File.ReadAllText(Path.Combine(__SOURCE_DIRECTORY__,"packaging_preview.html"))
    File.WriteAllText(Path.Combine(output,"index.html"),template.Replace("__PACKAGING_DATA__",json))
    printfn "%s" (Realization.json report.counts)

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--self-test"] -> selfTest ()
| ["--parent";parent;"--mass-properties";properties;"--output";output] -> experiment parent properties output
| _ -> invalidArg "arguments" "Expected --parent DIR --mass-properties JSON --output EMPTY_DIR, or --self-test."
