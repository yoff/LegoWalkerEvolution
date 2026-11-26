open System
open System.Globalization
open System.IO
open System.Text.Json
open System.Text.Json.Nodes
open LegoWalkers

let usage = """LegoWalkers: constrained Technic mechanism compiler

  generate --output DIR [--genome JSON] [--kind walker|leg|hinge|rigid|motor]
           [--pairs 1..3] [--ground 3|4] [--coupler 3|4] [--rocker 3|4]
           [--phase 0..3] [--seed INT]
  sweep    --output DIR [--limit 1..32] [--seed INT]
  catalog
  propose --pairs 1..3 --variant wired|remote --towing yes|no --seed INT [--attempt INT] [--parent JSON]
  motion --output DIR [--example four-bar|walker1|jansen | --program JSON] [--steps 36..3600] [--turns 1..32]
  motion-gallery --output DIR
  mutation-gallery --output DIR
  motif-gallery --output DIR
  realize --output DIR [--example four-bar|walker1 | --program JSON]
          [--genome JSON | --limit 1..32] [--steps 36..3600]
  realize-walker --output DIR [--example four-bar|walker1] [--feet 2..6 | --program JSON --genome JSON] [--leg-policy shared|independent] [--steps 36..3600]
  walker-edits --parent DIR [--steps 36..3600]
  share-walker-legs --parent DIR --output DIR --feet FOOT[,FOOT...] [--family NAME] [--steps 36..3600]
  migrate-walker-legs --parent DIR --output DIR [--steps 36..3600]
  clearance-walker --parent DIR --output DIR [--steps 36..3600]
  propose-walker --parent DIR --output DIR --seed INT [--attempt 0..63 | --edit ID] [--builder-policy JSON] [--limit 1..4] [--steps 36..3600]
  construct-walker --parent DIR --output DIR [--feedback JSON [--conflicts JSON]] [--builder-policy JSON] [--limit 1..32] [--steps 36..3600]
  build-walker --parent DIR --output DIR [--limit 1..32] [--steps 36..3600]
               [--foot-policy paired-plates|retain] [--foot-layers 1|2]
               [--target-placements JSON] [--ground-clearance-ldu NUMBER]
               [--max-roll-degrees NUMBER] [--max-pitch-degrees NUMBER]
               [--seen-requests JSON-ARRAY-OF-SCOPED-REQUEST-IDS]
    Ground options also apply to construct-walker without --feedback.
    Defaults: paired plates, 8 LDU non-foot clearance, upright nominal attitude.
    Writes throttled live proof/envelope snapshots and a recorded search-event stream.
  inner-feet-walker --parent DIR --output DIR [--steps 36..3600]
  foot-plates-walker --parent DIR --output DIR [--layers 1..2] [--steps 36..3600]
  grammar-walker --parent DIR --output DIR --conflicts JSON [--limit 1..32] [--steps 36..3600]
  builder-catalogue [--builder-policy JSON]

Span parameters are hole-to-hole distances in studs; phase is a crank
quarter-turn index. Sweeps enumerate valid discrete designs, not simulations.
Output directories must be empty. Use the Python adapter to check/evaluate.
"""

let parseOptions allowed (args: string list) =
    let rec loop acc = function
        | [] -> acc
        | key :: value :: rest when Set.contains key allowed && not (Map.containsKey key acc) ->
            loop (Map.add key value acc) rest
        | key :: _ -> invalidArg "arguments" $"Unknown, duplicate, or incomplete option: {key}"
    loop Map.empty args

let integer key fallback (options: Map<string, string>) =
    match Map.tryFind key options with
    | None -> fallback
    | Some text ->
        match Int32.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture) with
        | true, value -> value
        | _ -> invalidArg key $"Expected an integer for {key}: {text}"

let output options =
    match Map.tryFind "--output" options with
    | Some path when not (String.IsNullOrWhiteSpace path) -> Path.GetFullPath path
    | _ -> invalidArg "--output" "An explicit output directory is required."

let builderPolicy options =
    Map.tryFind "--builder-policy" options |> Option.map WalkerBuilder.readPolicy
    |> Option.defaultValue WalkerBuilder.defaultPolicy

let compile kind genome =
    match Grammar.compile kind genome with
    | Ok assembly -> assembly
    | Error errors -> invalidArg "genome" (String.concat Environment.NewLine errors)

let writeWalker = WalkerClearance.writeRealization

let constructionTrace options =
    let directory = output options
    if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
        invalidArg "--output" "Output directory must be empty."
    Directory.CreateDirectory directory |> ignore
    let events = ResizeArray<_>()
    fun stage status attempt detail ->
        events.Add {| stage = stage; status = status; attempt = attempt; detail = detail
                      created = DateTimeOffset.UtcNow.ToString("O") |}
        let path = Path.Combine(directory, "construction-progress.json")
        File.WriteAllText(path+".tmp", Realization.json {| schemaVersion = 1; events = events.ToArray() |}+"\n")
        File.Move(path+".tmp", path, true)

let buildWalker options =
    let trace = constructionTrace options
    let policy = builderPolicy options
    if policy<>WalkerBuilder.defaultPolicy then
        invalidArg "--builder-policy" "Motion-first packaging currently uses the compiled default construction policy."
    let limit = integer "--limit" 4 options
    if limit<1 || limit>32 then invalidArg "--limit" "Construction permits one to 32 exports."
    let parent = options |> Map.tryFind "--parent" |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate is required.")
    let unwrap result = Result.defaultWith (Realization.json >> invalidArg "--parent") result
    let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let genome = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
    let screened = Realization.screen program (integer "--steps" 72 options) |> unwrap
    let baseline = WalkerRealization.compile screened genome |> unwrap
    let pairedFeet =
        match Map.tryFind "--foot-policy" options |> Option.defaultValue "paired-plates" with
        | "paired-plates" -> true
        | "retain" -> false
        | _ -> invalidArg "--foot-policy" "Expected paired-plates or retain."
    let layers = integer "--foot-layers" 1 options
    if layers<1 || layers>2 || (not pairedFeet && layers<>1) then
        invalidArg "--foot-layers" "Choose one or two layers with paired-plates; retain does not change plate stacks."
    let source,footGates =
        if pairedFeet then WalkerFootPlates.prepareWithLayers layers screened baseline.Genome |> unwrap
        else baseline.Genome,[{Gate="foot-construction";Outcome="pass";Code="explicit-retained-feet"
                               Diagnostics=["Existing foot construction retained by explicit caller policy."]}]
    let request = WalkerPackaging.rebuildRequestWithFeet pairedFeet screened source
    let request = {request with FootConstructionGates=footGates}
    let request =
        match Map.tryFind "--target-placements" options with
        | None -> request
        | Some path ->
            let fields =
                match JsonNode.Parse(File.ReadAllText path) with
                | :? JsonObject as fields -> fields
                | _ -> invalidArg "--target-placements" "Expected an object mapping component IDs to finite x/y/z positions."
            let known = request.Targets |> List.map (fun target -> target.Placement.Id) |> Set.ofList
            let placements = fields |> Seq.map (fun field ->
                if not(known.Contains field.Key) then invalidArg "--target-placements" $"Unknown component {field.Key}."
                let point =
                    match field.Value with
                    | :? JsonObject as point when (point |> Seq.map _.Key |> Set.ofSeq)=Set.ofList ["x";"y";"z"] ->
                        Vec3.create (point["x"].GetValue<float>()) (point["y"].GetValue<float>()) (point["z"].GetValue<float>())
                    | _ -> invalidArg "--target-placements" $"Component {field.Key} requires exactly x, y and z."
                if not(List.forall Double.IsFinite [point.X;point.Y;point.Z]) then
                    invalidArg "--target-placements" "Component positions must be finite."
                field.Key,point) |> Map.ofSeq
            let targets = request.Targets |> List.map (fun target ->
                match placements |> Map.tryFind target.Placement.Id with
                | None -> target
                | Some point -> {target with Placement={target.Placement with Position=point}})
            {request with Targets=targets}
    let number name fallback =
        options |> Map.tryFind name |> Option.map (fun value ->
            Double.Parse(value,Globalization.CultureInfo.InvariantCulture)) |> Option.defaultValue fallback
    let ground = {WalkerMotionSpace.defaultGroundClearance with
                    MinimumNonFootClearanceLdu=number "--ground-clearance-ldu" 8.0
                    MaximumRollRadians=number "--max-roll-degrees" 0.0*Math.PI/180.0
                    MaximumPitchRadians=number "--max-pitch-degrees" 0.0*Math.PI/180.0}
    let request = {request with GroundClearance=Some ground}
    let directory = output options
    WalkerBuilder.write directory policy
    File.WriteAllText(Path.Combine(directory,"request.json"),Realization.json request+"\n")
    let oracleScope = "motion-first-build-awaiting-exact-admission-v1"
    let initial = WalkerConstructionTheory.start request oracleScope 512 64
    let requestId = initial.Scope
    let seenRequests =
        match Map.tryFind "--seen-requests" options with
        | None -> []
        | Some value ->
            match JsonNode.Parse value with
            | :? JsonArray as values -> values |> Seq.map (fun v -> v.GetValue<string>()) |> Seq.toList
            | _ -> invalidArg "--seen-requests" "Expected an array of effective construction request identities."
    let timer = Diagnostics.Stopwatch.StartNew()
    let mutable lastSnapshot = -1.0
    let mutable observedState = initial
    let mutable observedProof = initial.Frontier.Head
    let mutable inspection: WalkerConstructionInspection option = None
    let mutable detail = "Opening the construction proof."
    let writeSnapshot force =
        if force || timer.Elapsed.TotalSeconds-lastSnapshot>=0.5 then
            let stamp = DateTimeOffset.UtcNow.ToString("O")
            let document = {|
                schemaVersion=1;requestId=requestId;updated=stamp;elapsedSeconds=timer.Elapsed.TotalSeconds
                status=observedState.Status;detail=detail;proof=observedProof
                openGoals=ConstructionSearch.openGoals observedProof;inspection=inspection
                search={|visited=observedState.Visited;maximumStates=observedState.MaximumStates
                         frontier=observedState.Frontier.Length;pruned=observedState.Pruned;discarded=observedState.Discarded|}
                authority="Live partial construction and sampled stock envelopes; not a qualified assembly or physics."
                dynamicTrials=0
            |}
            let path = Path.Combine(directory,"construction-live.json")
            File.WriteAllText(path+".tmp",Realization.json document+"\n")
            File.Move(path+".tmp",path,true)
            let event = {|
                updated=stamp;elapsedSeconds=timer.Elapsed.TotalSeconds;status=observedState.Status;detail=detail
                node=ConstructionSearch.nodeId observedProof;visited=observedState.Visited
                goal=inspection |> Option.map _.Goal
                ``checked``=inspection |> Option.map _.Checked;rejected=inspection |> Option.map _.Rejected
            |}
            File.AppendAllText(Path.Combine(directory,"construction-events.jsonl"),Realization.json event |> fun text -> text.Replace("\n","").Replace("\r","")+"\n")
            lastSnapshot <- timer.Elapsed.TotalSeconds
    let observe state proof outcome =
        if ConstructionSearch.nodeId proof<>ConstructionSearch.nodeId observedProof then inspection<-None
        observedState <- state
        observedProof <- proof
        detail <-
            match outcome with
            | None ->
                ConstructionSearch.openGoals proof |> List.tryFind (fun goal -> not goal.External)
                |> Option.map (fun goal -> $"Expanding {goal.Id} ({goal.Kind}).")
                |> Option.defaultValue "Verifying the locally constructed witness."
            | Some(Branch(goal,choices)) -> $"{goal}: {choices.Length} applicable productions."
            | Some(Blocked gate) -> gate.Code+": "+String.concat "; " gate.Diagnostics
            | Some(Built _) -> "Local construction witness found; external admission is still required."
        writeSnapshot false
    let inspect produce =
        if timer.Elapsed.TotalSeconds-lastSnapshot>=0.5 then
            inspection <- Some(produce ())
            detail <- $"Checking {inspection.Value.Goal}: {inspection.Value.Rule}."
            writeSnapshot false
    writeSnapshot true
    trace "planning" "running" 0 "Reserving the complete geared gait before choosing hardware, stock axles and support routes."
    let duplicate = List.contains requestId seenRequests
    let state,construction =
        if duplicate then
            let gate =
                {Gate="construction-search";Outcome="reject";Code="duplicate-effective-request"
                 Diagnostics=["This effective request was already searched in the current build obligation; no repeated search was run."]}
            {initial with Status="duplicate-request";Rejections=[gate]},None
        else WalkerConstructionTheory.advanceObserved observe inspect request oracleScope initial
    observedState <- state
    state.Pending |> Option.iter (fun proof -> observedProof<-proof)
    detail <- if duplicate then "Duplicate effective request skipped." else $"Search stopped: {state.Status}."
    writeSnapshot true
    File.WriteAllText(Path.Combine(directory,"construction-search.json"),Realization.json state+"\n")
    let generated =
        construction |> Option.toList |> List.map (fun built ->
            let destination = writeWalker (Path.Combine(directory,"candidate-001")) built.Screened built.Realized
            let proof = state.Pending |> Option.defaultWith (fun () -> invalidOp "A construction witness requires its pending proof.")
            File.WriteAllText(Path.Combine(destination,"construction-proof.json"),Realization.json proof+"\n")
            let space = WalkerMotionSpace.reserveWithGround request.GroundClearance built.Screened built.Realized.Genome |> unwrap
            File.WriteAllText(Path.Combine(destination,"motion-space.json"),Realization.json (WalkerMotionSpace.describe space)+"\n")
            let groundPlan = WalkerMotionSpace.groundPlan ground built.Screened built.Realized.Genome
            File.WriteAllText(Path.Combine(destination,"ground-plan.json"),Realization.json groundPlan+"\n")
            let choice = {WalkerConstruction.describe screened baseline.Genome built.Realized.Genome with Motif="motion-first-hardware-construction"}
            WalkerConstruction.writeWithPolicy policy destination choice
            {|directory="candidate-001";assemblyId=built.Realized.AssemblyId;genotypeId=built.Realized.GenotypeId|})
    let gates =
        if generated.IsEmpty then
            state.Rejections @ [{Gate="motion-space";Outcome="reject";Code="no-bounded-motion-first-construction";
                                 Diagnostics=[$"Search stopped as {state.Status} after {state.Visited}/512 states; no assembly exported."]}]
        else [{Gate="motion-space";Outcome="pass";Code="stock-envelope-placement-constraints";
               Diagnostics=["Hardware and support routes selected outside reserved sampled motion; exact complete-assembly phase admission remains mandatory."]}]
    let summary = {|
        schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=screened.MechanismId
        constructionVersion=WalkerConstruction.version;builderPolicyId=WalkerBuilder.policyId policy
        parentGenotypeId=baseline.GenotypeId;operation="construct-realization"
        changed=[{|path="construction";before=[baseline.GenotypeId];after=[WalkerConstruction.version;WalkerMotionSpace.version]|}]
        gates=screened.Decisions @ request.FootConstructionGates @ gates;generated=generated
        rejected=if generated.IsEmpty then [List.last gates] else []
        counts={|proposals=1;realizationAttempts=(if duplicate then 0 else 1);exported=generated.Length
                 rejected=(if generated.IsEmpty && not duplicate then 1 else 0);duplicates=(if duplicate then 1 else 0);dynamicTrials=0|}
        motionFirst={|version=WalkerMotionSpace.version;theoryVersion=WalkerConstructionTheory.version
                      packagingVersion=WalkerPackaging.version;maximumStates=512;maximumFrontier=64
                      maximumTargetAssignments=32;visitedStates=state.Visited;status=state.Status;requestId=requestId
                      qualification="Pending exact reconstruction, complete-assembly phase clearance and separately authorized dynamics."|}
    |}
    File.WriteAllText(Path.Combine(directory,"candidates.json"),Realization.json summary+"\n")
    trace "building" (if generated.IsEmpty then "rejected" else "complete") 0 $"Exported {generated.Length} motion-first construction witnesses; no dynamics."
    printfn "%s" (Realization.json summary)
    if generated.IsEmpty then 1 else 0

[<EntryPoint>]
let main argv =
    try
        match Array.toList argv with
        | [] | ["help"] | ["--help"] -> printf "%s" usage; 0
        | ["catalog"] ->
            for KeyValue(id, spec) in Catalog.parts do
                printfn "%-14s %-24s %d mechanical ports" id
                    (match spec.Connector with Some c -> string c | None -> "structural") spec.Ports.Length
            0
        | "builder-catalogue" :: args ->
            let options = parseOptions (Set.ofList ["--builder-policy"]) args
            printfn "%s" (Realization.json (WalkerBuilder.describe (builderPolicy options)))
            0
        | "motion" :: args ->
            let options = parseOptions (Set.ofList ["--output"; "--example"; "--program"; "--steps"; "--turns"]) args
            if options.ContainsKey "--example" && options.ContainsKey "--program" then
                invalidArg "motion" "Choose an example or an input program, not both."
            let program =
                match Map.tryFind "--program" options with
                | Some path -> MotionExport.readProgram path
                | None ->
                    match MotionExamples.named (Map.tryFind "--example" options |> Option.defaultValue "four-bar") with
                    | Ok program -> program
                    | Error error -> invalidArg "--example" error
            let directory, frames =
                MotionExport.writeTurns (output options) program (integer "--steps" 360 options) (integer "--turns" 1 options)
            printfn "%s: %d bodies, %d hinges, one motor input, %d solved frames (kinematics only)"
                directory program.Bodies.Length program.Hinges.Length frames
            0
        | "motion-gallery" :: args ->
            let options = parseOptions (Set.ofList ["--output"]) args
            let directory = output options
            for name in MotionExport.writeTestGallery directory do
                printfn "%s" (Path.Combine(directory, name))
            let program, _ = WalkerSeeds.fourBar 4
            let screened = Realization.screen program 180 |> Result.defaultWith (fun g -> invalidOp (Realization.json g))
            let path = Path.Combine(directory, "whole-walker.svg")
            File.WriteAllText(path, MotionExport.cycleSvg program screened.Graph screened.Frames)
            printfn "%s" path
            0
        | "motif-gallery" :: args ->
            let options = parseOptions (Set.ofList ["--output"]) args
            let directory = output options
            WalkerMotifGallery.write directory
            printfn "%s" (Path.Combine(directory, "gallery.json"))
            0
        | "mutation-gallery" :: args ->
            let options = parseOptions (Set.ofList ["--output"]) args
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
            let program, genome = WalkerSeeds.fourBar 4
            let parent = Realization.screen program 72 |> unwrap
            let decoded = WalkerRealization.compile parent genome |> unwrap
            let find operation source physical =
                let choices = WalkerEvolution.mutations source physical @ WalkerEvolution.topologyMutations source physical
                              @ WalkerEvolution.physicalMutations source physical
                choices
                |> List.filter (fun m -> m.Operation = operation)
                |> List.tryPick (fun mutation ->
                    match Realization.screen mutation.Program 72 with
                    | Error _ -> None
                    | Ok child ->
                        match WalkerEvolution.realize source physical mutation child 1 with
                        | Ok [choice] ->
                            match WalkerRealization.compile child choice with
                            | Ok robot when (Realization.screenAssemblyGeometry child robot.Assembly).Outcome <> "reject" ->
                                Some (mutation, child, robot)
                            | _ -> None
                        | _ -> None)
                |> Option.defaultWith (fun () -> invalidOp $"No realizable gallery witness for {operation}.")
            let write operation (source: ScreenedMotion) sourceRobot (mutation: WalkerMutation) (child: ScreenedMotion) childRobot =
                let target = Path.Combine(directory, operation)
                let turns = if operation = "gear-reduction" then 5 else 1
                MotionExport.writeTurns (Path.Combine(target, "parent")) source.Program 72 turns |> ignore
                MotionExport.writeTurns (Path.Combine(target, "child")) child.Program 72 turns |> ignore
                writeWalker (Path.Combine(target, "lego-parent")) source sourceRobot |> ignore
                writeWalker (Path.Combine(target, "lego-child")) child childRobot |> ignore
                File.WriteAllText(Path.Combine(target, "mutation.json"),
                                  Realization.json {| operation = operation; changed = mutation.Changes
                                                      mode = "solved-kinematics-not-dynamics" |})
                printfn "%s" target
            for operation in ["move-hinge-site"; "input-phase"; "branch-phase"; "add-dyad"] do
                let mutation, child, robot = find operation parent decoded.Genome
                write operation parent decoded mutation child robot
            let _, larger, physical = find "add-dyad" parent decoded.Genome
            let removal, smaller, robot = find "remove-dyad" larger physical.Genome
            write "remove-dyad" larger physical removal smaller robot
            for operation in ["lengthen-foot";"bend-foot";"gear-reduction";"battery-layout";"receiver-layout";"motor-layout"] do
                let mutation, child, robot = find operation parent decoded.Genome
                write operation parent decoded mutation child robot
            let p1,g1 = Walker1Seeds.walker 2
            let s1 = Realization.screen p1 72 |> unwrap
            let r1 = WalkerRealization.compile s1 g1 |> unwrap
            let _,s3,r3 = find "duplicate-branch" s1 r1.Genome
            let duplication,s4,r4 = find "duplicate-branch" s3 r3.Genome
            write "duplicate-branch" s3 r3 duplication s4 r4
            let removal,s3b,r3b = find "remove-branch" s4 r4.Genome
            write "remove-branch" s4 r4 removal s3b r3b
            0
        | "realize" :: args ->
            let options = parseOptions (Set.ofList ["--output"; "--example"; "--program"; "--genome"; "--limit"; "--steps"]) args
            if options.ContainsKey "--example" && options.ContainsKey "--program" then
                invalidArg "realize" "Choose an example or a program, not both."
            if options.ContainsKey "--genome" && options.ContainsKey "--limit" then
                invalidArg "realize" "A complete realization genome decodes exactly one assembly; do not specify --limit."
            let program =
                match Map.tryFind "--program" options with
                | Some path -> MotionExport.readProgramDocument path
                | None -> MotionExamples.named (Map.tryFind "--example" options |> Option.defaultValue "four-bar") |> Result.defaultWith (fun e -> invalidArg "--example" e)
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            Directory.CreateDirectory directory |> ignore
            let gates = ResizeArray<GateDecision>()
            let rejected = ResizeArray<_>()
            let generated = ResizeArray<_>()
            let seen = Collections.Generic.HashSet<string>()
            let mutable attempts = 0
            match Realization.screen program (integer "--steps" 360 options) with
            | Error gate -> gates.Add gate
            | Ok screened ->
                for gate in screened.Decisions do gates.Add gate
                let proposals =
                    match Map.tryFind "--genome" options with
                    | Some path -> Ok [Realization.readGenome path]
                    | None -> Realization.propose screened (integer "--limit" 4 options)
                match proposals with
                | Error gate -> gates.Add gate
                | Ok genomes ->
                    for genome in genomes do
                        attempts <- attempts+1
                        match Realization.compile screened genome with
                        | Error gate -> rejected.Add {| attempt = attempts; genome = genome; decision = gate |}
                        | Ok realized ->
                            let geometry = Realization.screenGeometry screened realized
                            if geometry.Outcome = "reject" then
                                rejected.Add {| attempt = attempts; genome = genome; decision = geometry |}
                            elif seen.Add realized.AssemblyId then
                                let name = $"candidate-{attempts:D3}"
                                Realization.write (Path.Combine(directory, name)) screened realized |> ignore
                                generated.Add {| directory = name; assemblyId = realized.AssemblyId; genotypeId = realized.GenotypeId |}
            let summary = {|
                schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = Realization.mechanismId program
                footCount = Motion.footCount program
                gates = gates.ToArray(); generated = generated.ToArray(); rejected = rejected.ToArray()
                counts = {| proposals = 1; realizationAttempts = attempts; rejected = rejected.Count
                            duplicates = attempts-rejected.Count-generated.Count; exported = generated.Count; dynamicTrials = 0 |}
            |}
            File.WriteAllText(Path.Combine(directory, "candidates.json"), Realization.json summary+"\n")
            printfn "%s" (Realization.json summary)
            if generated.Count = 0 then 1 else 0
        | "realize-walker" :: args ->
            let options = parseOptions (Set.ofList ["--output"; "--example"; "--feet"; "--program"; "--genome"; "--steps"; "--leg-policy"]) args
            let supplied = options.ContainsKey "--program" || options.ContainsKey "--genome"
            if supplied && (not (options.ContainsKey "--program" && options.ContainsKey "--genome") || options.ContainsKey "--feet" || options.ContainsKey "--example" || options.ContainsKey "--leg-policy") then
                invalidArg "realize-walker" "Supply both --program and --genome, or choose a --feet seed."
            let program, genome =
                if supplied then MotionExport.readProgramDocument options["--program"], WalkerRealization.readGenome options["--genome"]
                else
                    let shared =
                        match Map.tryFind "--leg-policy" options |> Option.defaultValue "shared" with
                        | "shared" -> true | "independent" -> false
                        | _ -> invalidArg "--leg-policy" "Expected shared or independent."
                    match Map.tryFind "--example" options |> Option.defaultValue "four-bar" with
                    | "four-bar" -> (if shared then WalkerSeeds.sharedFourBar else WalkerSeeds.fourBar) (integer "--feet" 4 options)
                    | "walker1" -> (if shared then Walker1Seeds.sharedWalker else Walker1Seeds.walker) (integer "--feet" 2 options)
                    | _ -> invalidArg "--example" "Expected four-bar or walker1."
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            let gates = ResizeArray<GateDecision>()
            let rejected, generated = ResizeArray<_>(), ResizeArray<_>()
            match Realization.screen program (integer "--steps" 360 options) with
            | Error gate -> gates.Add gate
            | Ok screened ->
                gates.AddRange screened.Decisions
                match WalkerRealization.compile screened genome with
                | Error gate -> rejected.Add {| attempt = 1; decision = gate |}
                | Ok realized ->
                    let geometry = Realization.screenAssemblyGeometry screened realized.Assembly
                    if geometry.Outcome = "reject" then rejected.Add {| attempt = 1; decision = geometry |}
                    else
                        writeWalker (Path.Combine(directory, "candidate-001")) screened realized |> ignore
                        generated.Add {| directory = "candidate-001"; assemblyId = realized.AssemblyId; genotypeId = realized.GenotypeId |}
            Directory.CreateDirectory directory |> ignore
            let attempts = if gates |> Seq.exists (fun g -> g.Outcome = "reject") then 0 else 1
            let summary = {|
                schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = Realization.mechanismId program
                footCount = Motion.footCount program
                gates = gates.ToArray(); generated = generated.ToArray(); rejected = rejected.ToArray()
                counts = {| proposals = 1; realizationAttempts = attempts; rejected = rejected.Count
                            duplicates = 0; exported = generated.Count; dynamicTrials = 0 |}
            |}
            File.WriteAllText(Path.Combine(directory, "candidates.json"), Realization.json summary+"\n")
            printfn "%s" (Realization.json summary)
            if generated.Count = 0 then 1 else 0
        | "grammar-walker" :: args ->
            let options = parseOptions (Set.ofList ["--parent"; "--output"; "--conflicts"; "--limit"; "--steps"]) args
            let required key =
                Map.tryFind key options |> Option.defaultWith (fun () -> invalidArg key $"Required option: {key}")
            let parentPath = required "--parent"
            let conflicts = WalkerConnectionGrammar.readConflicts (required "--conflicts")
            let program = MotionExport.readProgramDocument (Path.Combine(parentPath,"program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(parentPath,"realization.json"))
            let screened =
                Realization.screen program (integer "--steps" 72 options)
                |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let baseline =
                WalkerRealization.compile screened genome
                |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let search = WalkerConnectionGrammar.derive screened baseline.Genome conflicts (integer "--limit" 8 options)
            let summary = WalkerConnectionGrammar.writeBatch (output options) screened baseline conflicts search
            printfn "%s" (Realization.json summary)
            if summary.generated.Length = 0 then 1 else 0
        | "clearance-walker" :: args ->
            let options = parseOptions (Set.ofList ["--parent"; "--output"; "--steps"]) args
            let parentPath = Map.tryFind "--parent" options |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate is required.")
            let program = MotionExport.readProgramDocument (Path.Combine(parentPath,"program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(parentPath,"realization.json"))
            let screened = Realization.screen program (integer "--steps" 72 options)
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let realized = WalkerRealization.compile screened genome
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            Directory.CreateDirectory directory |> ignore
            let _,report = WalkerClearance.inspect screened realized.Genome
            WalkerClearance.write directory report
            printfn "%s" (Realization.json report)
            0
        | command :: args when command="inner-feet-walker" || command="foot-plates-walker" ->
            let options = parseOptions (Set.ofList (["--parent";"--output";"--steps"] @
                                        if command="foot-plates-walker" then ["--layers"] else [])) args
            let parent = Map.tryFind "--parent" options |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate is required.")
            let unwrap result = Result.defaultWith (Realization.json >> invalidArg "--parent") result
            let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
            let screened = Realization.screen program (integer "--steps" 72 options) |> unwrap
            let baseline = WalkerRealization.compile screened (WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))) |> unwrap
            let genome,operation =
                if command="foot-plates-walker" then
                    WalkerFootPlates.add screened baseline.Genome (integer "--layers" 1 options) |> unwrap,
                    WalkerFootPlates.version
                else WalkerInnerFeet.add screened baseline.Genome |> unwrap,WalkerInnerFeet.version
            let realized = WalkerRealization.compile screened genome |> unwrap
            let directory = output options
            if Directory.Exists directory && not(Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            writeWalker (Path.Combine(directory,"candidate-001")) screened realized |> ignore
            let summary = {|
                schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=screened.MechanismId
                parentGenotypeId=baseline.GenotypeId;operation=operation
                gates=screened.Decisions;rejected=List.empty<GateDecision>
                generated=[{|directory="candidate-001";assemblyId=realized.AssemblyId;genotypeId=realized.GenotypeId|}]
                counts={|proposals=1;realizationAttempts=1;exported=1;rejected=0;duplicates=0;dynamicTrials=0|}
            |}
            File.WriteAllText(Path.Combine(directory,"candidates.json"),Realization.json summary+"\n")
            printfn "%s" (Realization.json summary)
            0
        | command :: args when command="build-walker" || (command="construct-walker" && not(List.contains "--feedback" args)) ->
            let options = parseOptions (Set.ofList ["--parent";"--output";"--builder-policy";"--limit";"--steps";
                                                   "--foot-policy";"--foot-layers";"--target-placements";"--seen-requests";"--ground-clearance-ldu";"--max-roll-degrees";"--max-pitch-degrees"]) args
            buildWalker options
        | "construct-walker" :: args ->
            let options = parseOptions (Set.ofList ["--parent"; "--output"; "--feedback"; "--conflicts"; "--builder-policy"; "--limit"; "--steps"]) args
            let trace = constructionTrace options
            trace "planning" "running" 0 "Reading the parent motion and checking its existing realization."
            let policy = builderPolicy options
            let parentPath = options |> Map.tryFind "--parent" |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate directory is required.")
            let sourceManifest = JsonNode.Parse(File.ReadAllText(Path.Combine(parentPath, "manifest.json")))
            let sourceMotion = sourceManifest["motion"]
            let program = MotionExport.readProgramDocument (Path.Combine(parentPath, "program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(parentPath, "realization.json"))
            let screened = Realization.screen program (integer "--steps" 72 options)
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let baseline = WalkerRealization.compile screened genome
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let feedback = Map.tryFind "--feedback" options |> Option.map WalkerConstruction.readFeedback
            let conflicts = Map.tryFind "--conflicts" options |> Option.map WalkerConnectionGrammar.readConflicts
            if conflicts.IsSome && (feedback.IsNone || not (List.contains feedback.Value.Kind ["initial-self-interference";"motion-self-interference"])) then
                invalidArg "--conflicts" "Connection grammar conflicts require measured self-interference feedback."
            if conflicts |> Option.exists (fun c -> isNull (box c) || isNull (box c.Pairs) || c.Pairs.IsEmpty) then
                invalidArg "--conflicts" "Connection grammar repairs require at least one reported contact pair."
            let grammar =
                trace "planning" "running" 0 "Selecting feedback-driven link layers and compatible connector stacks."
                conflicts |> Option.map (fun pairs ->
                    WalkerConnectionGrammar.derive screened baseline.Genome pairs (integer "--limit" 4 options))
            let layouts =
                match conflicts,feedback with
                | Some _,Some hints ->
                    WalkerConstruction.measuredLayouts screened baseline.Genome hints.Parts (integer "--limit" 4 options)
                | _ -> []
            let choices =
                match grammar,feedback with
                | Some search,_ ->
                    layouts @ (search.Choices |> List.map _.Construction)
                    |> List.distinctBy (fun c -> WalkerRealization.json c.Genome)
                    |> List.truncate (integer "--limit" 4 options)
                | None,Some hints -> WalkerConstruction.repairsWithPolicy policy screened baseline.Genome hints (integer "--limit" 4 options)
                | None,None -> WalkerConstruction.alternativesWithPolicy policy screened baseline.Genome (integer "--limit" 4 options)
            let directory = output options
            trace "planning" "complete" 0 $"Selected {choices.Length} bounded construction alternatives."
            WalkerBuilder.write directory policy
            feedback |> Option.iter (fun hints ->
                File.WriteAllText(Path.Combine(directory, "feedback.json"), Realization.json hints+"\n"))
            conflicts |> Option.iter (fun pairs ->
                File.WriteAllText(Path.Combine(directory, "conflicts.json"), Realization.json pairs+"\n"))
            grammar |> Option.iter (fun search ->
                File.WriteAllText(Path.Combine(directory, "grammar-search.json"),
                    Realization.json (WalkerConnectionGrammar.describeSearch search)+"\n"))
            let generated, rejected = ResizeArray<_>(), ResizeArray<_>()
            let seen = Collections.Generic.HashSet<string>()
            let mutable attempts = 0
            for choice in choices do
                attempts <- attempts+1
                trace "building" "running" attempts $"Compiling {choice.Motif}; pivot {choice.Pivot}; layer offset {choice.LayerOffset} LDU."
                match WalkerRealization.compile screened choice.Genome with
                | Error gate ->
                    rejected.Add {| attempt = attempts; decision = gate |}
                    trace "building" "rejected" attempts (Realization.json gate)
                | Ok realized ->
                    let geometry = Realization.screenAssemblyGeometry screened realized.Assembly
                    if geometry.Outcome = "reject" then
                        rejected.Add {| attempt = attempts; decision = geometry |}
                        trace "building" "rejected" attempts (Realization.json geometry)
                    elif seen.Add realized.AssemblyId then
                        let name = $"candidate-{attempts:D3}"
                        let destination = writeWalker (Path.Combine(directory, name)) screened realized
                        WalkerConstruction.writeWithPolicy policy destination choice
                        WalkerConstruction.writeMotionEvidence destination screened baseline.Genome choice.Genome
                        grammar |> Option.iter (fun search ->
                            search.Choices |> List.tryFind (fun c -> c.Construction = choice)
                            |> Option.iter (WalkerConnectionGrammar.writeDerivation destination))
                        generated.Add {| directory = name; assemblyId = realized.AssemblyId; genotypeId = realized.GenotypeId |}
                        trace "building" "complete" attempts $"Exported {name}; exact reconstruction and dynamics still required."
                    else trace "building" "duplicate" attempts "Physical assembly already exported in this proposal."
            let summary = {|
                schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = screened.MechanismId
                constructionVersion = WalkerConstruction.version
                grammarVersion = grammar |> Option.map (fun _ -> WalkerConnectionGrammar.version)
                builderPolicyId = WalkerBuilder.policyId policy
                parentGenotypeId = sourceMotion["genotypeId"].GetValue<string>()
                operation = if feedback.IsSome then "repair-realization" else "construct-realization"
                changed = [| {| path = "construction"; before = [baseline.GenotypeId]; after = [WalkerConstruction.version] |} |]
                gates = screened.Decisions @ (
                    if choices.IsEmpty then
                        [{ Gate = "lego-realizability"; Outcome = "reject"
                           Code = if feedback |> Option.exists (fun f -> f.Kind = "motion-clearance") then "no-improving-motion-clearance-repair" else "no-compatible-construction-repair"
                           Diagnostics = ["No supported improving repair with sufficient sampling preserves the mechanism and required connection coverage."] }]
                    else [])
                generated = generated.ToArray(); rejected = rejected.ToArray()
                counts = {| proposals = 1; realizationAttempts = attempts; rejected = rejected.Count
                            duplicates = attempts-rejected.Count-generated.Count; exported = generated.Count; dynamicTrials = 0 |}
            |}
            File.WriteAllText(Path.Combine(directory, "candidates.json"), Realization.json summary+"\n")
            trace "building" (if generated.Count=0 then "rejected" else "complete") 0 $"Finished {attempts} attempts; exported {generated.Count}; rejected {rejected.Count}."
            printfn "%s" (Realization.json summary)
            if generated.Count = 0 then 1 else 0
        | command :: args when command="share-walker-legs" || command="migrate-walker-legs" ->
            let options = parseOptions (Set.ofList ["--parent";"--output";"--feet";"--family";"--steps"]) args
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            let path = options |> Map.tryFind "--parent" |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate directory is required.")
            if command="migrate-walker-legs" && (options.ContainsKey "--feet" || options.ContainsKey "--family") then
                invalidArg command "Automatic conversion does not accept --feet or --family."
            let program = MotionExport.readProgramDocument (Path.Combine(path,"program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(path,"realization.json"))
            let screened = Realization.screen program (integer "--steps" 72 options)
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            WalkerRealization.compile screened genome
            |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate)) |> ignore
            let migration =
                if command="migrate-walker-legs" then WalkerLegFamilies.migrate screened genome
                else
                    let feet =
                        options |> Map.tryFind "--feet" |> Option.map (fun text -> text.Split(',') |> Array.toList)
                        |> Option.defaultWith (fun () -> invalidArg "--feet" "Choose the logical feet to bind to one template.")
                    let family = Map.tryFind "--family" options |> Option.defaultValue "legs"
                    let shared = WalkerLegFamilies.adopt family feet program genome
                    {WalkerLegFamilies.Migration.Genome=shared;Status="explicitly-bound"
                     Families=shared.LegFamilies.Value;Unmatched=[]}
            let shared = migration.Genome
            let realized = WalkerRealization.compile screened shared
                           |> Result.defaultWith (fun gate -> invalidArg "--feet" (Realization.json gate))
            let destination = writeWalker (Path.Combine(directory,"candidate-001")) screened realized
            let summary = {|
                schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=screened.MechanismId
                operation="share-leg-template";dynamicTrials=0;gates=screened.Decisions
                generated=[{|directory="candidate-001";assemblyId=realized.AssemblyId;genotypeId=realized.GenotypeId|}]
                rejected=Array.empty<GateDecision>
                counts={|proposals=1;realizationAttempts=1;rejected=0;duplicates=0;exported=1;dynamicTrials=0|}
            |}
            File.WriteAllText(Path.Combine(directory,"candidates.json"),Realization.json summary+"\n")
            File.WriteAllText(Path.Combine(directory,"leg-family-conversion.json"),Realization.json {|
                schemaVersion=1;status=migration.Status;families=migration.Families
                unmatched=migration.Unmatched;dynamicTrials=0
            |}+"\n")
            printfn "%s" (Realization.json {|directory=destination;assemblyId=realized.AssemblyId;genotypeId=realized.GenotypeId;dynamicTrials=0|})
            0
        | "walker-edits" :: args ->
            let options = parseOptions (Set.ofList ["--parent"; "--steps"; "--builder-policy"]) args
            let path = options |> Map.tryFind "--parent" |> Option.defaultWith (fun () -> invalidArg "--parent" "A native candidate directory is required.")
            let program = MotionExport.readProgramDocument (Path.Combine(path, "program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(path, "realization.json"))
            let screened = Realization.screen program (integer "--steps" 72 options)
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let realized = WalkerRealization.compile screened genome
                           |> Result.defaultWith (fun gate -> invalidArg "--parent" (Realization.json gate))
            let edits =
                WalkerEvolution.editsWithPolicy (builderPolicy options) screened realized.Genome
                |> List.map (fun (id,level,mutation) ->
                    {| id = id; level = level; operation = mutation.Operation; changed = mutation.Changes
                       motionUnchanged = mutation.Program = program |})
            printfn "%s" (Realization.json {| schemaVersion = 1; edits = edits; dynamicTrials = 0 |})
            0
        | "propose-walker" :: args ->
            let options = parseOptions (Set.ofList ["--parent"; "--output"; "--seed"; "--attempt"; "--edit"; "--builder-policy"; "--limit"; "--steps"]) args
            let trace = constructionTrace options
            trace "planning" "running" 0 "Reading the parent motion and checking its existing realization."
            let policy = builderPolicy options
            let limit = integer "--limit" 2 options
            if limit < 1 || limit > 4 then invalidArg "--limit" "A native proposal permits one to four physical realizations."
            let parentPath = options |> Map.tryFind "--parent" |> Option.defaultWith (fun () -> invalidArg "--parent" "A complete native parent directory is required.")
            let sourceManifest = JsonNode.Parse(File.ReadAllText(Path.Combine(parentPath, "manifest.json")))
            let sourceMotion = sourceManifest["motion"]
            let steps = integer "--steps" 72 options
            let program = MotionExport.readProgramDocument (Path.Combine(parentPath, "program.json"))
            let genome = WalkerRealization.readGenome (Path.Combine(parentPath, "realization.json"))
            let parent = Realization.screen program steps |> Result.defaultWith (fun g -> invalidArg "--parent" (Realization.json g))
            let decoded = WalkerRealization.compile parent genome |> Result.defaultWith (fun g -> invalidArg "--parent" (Realization.json g))
            let seed, attempt = integer "--seed" 0 options, integer "--attempt" 0 options
            trace "planning" "running" 0 "Enumerating applicable mutation families and selecting the requested or seeded edit."
            let mutation, selection =
                match Map.tryFind "--edit" options with
                | None -> WalkerEvolution.chooseWithSelection parent decoded.Genome seed attempt
                | Some edit ->
                    let edits = WalkerEvolution.editsWithPolicy policy parent decoded.Genome
                    let _,_,chosen =
                        edits |> List.tryFind (fun (id,_,_) -> id = edit)
                        |> Option.defaultWith (fun () -> invalidArg "--edit" "Edit is unavailable for this parent; refresh the edit list.")
                    chosen, {| policy = "manual-applicable-edit-v1"; selectedFamily = chosen.Operation
                               availableFamilies = edits |> List.groupBy (fun (_,_,m) -> m.Operation)
                                                   |> List.map (fun (operation, choices) -> {| operation = operation; edits = choices.Length |}) |}
            let directory = output options
            trace "planning" "complete" 0 $"Selected {mutation.Operation}; screening its motion constraints."
            WalkerBuilder.write directory policy
            let gates, rejected, generated = ResizeArray<GateDecision>(), ResizeArray<_>(), ResizeArray<_>()
            let seen = Collections.Generic.HashSet<string>()
            let mutable attempts = 0
            match Realization.screen mutation.Program steps with
            | Error gate ->
                gates.Add gate
                trace "planning" "rejected" 0 (Realization.json gate)
            | Ok child ->
                gates.AddRange child.Decisions
                trace "building" "running" 0 "Realizing the selected motion with supported stock, link layers and connector alternatives."
                match WalkerEvolution.realizeWithPolicy policy parent decoded.Genome mutation child limit with
                | Error gate ->
                    gates.Add gate
                    trace "building" "rejected" 0 (Realization.json gate)
                | Ok variants ->
                    for choice in variants do
                        attempts <- attempts+1
                        trace "building" "running" attempts "Compiling the physical alternative and checking its assembly geometry."
                        match WalkerRealization.compile child choice with
                        | Error gate ->
                            rejected.Add {| attempt = attempts; decision = gate |}
                            trace "building" "rejected" attempts (Realization.json gate)
                        | Ok realized ->
                            let geometry = Realization.screenAssemblyGeometry child realized.Assembly
                            if geometry.Outcome = "reject" then
                                rejected.Add {| attempt = attempts; decision = geometry |}
                                trace "building" "rejected" attempts (Realization.json geometry)
                            elif seen.Add realized.AssemblyId then
                                let name = $"candidate-{attempts:D3}"
                                let destination = writeWalker (Path.Combine(directory, name)) child realized
                                WalkerConstruction.writeWithPolicy policy destination (WalkerConstruction.describe child variants.Head choice)
                                WalkerConstruction.writeMotionEvidence destination child variants.Head choice
                                generated.Add {| directory = name; assemblyId = realized.AssemblyId; genotypeId = realized.GenotypeId |}
                                trace "building" "complete" attempts $"Exported {name}; exact reconstruction and dynamics still required."
                            else trace "building" "duplicate" attempts "Physical assembly already exported in this proposal."
            let summary = {|
                schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = Realization.mechanismId mutation.Program
                mutationVersion = WalkerEvolution.version
                parentGenotypeId = sourceMotion["genotypeId"].GetValue<string>()
                seed = seed; attempt = attempt; selectedEdit = Map.tryFind "--edit" options
                constructionVersion = WalkerConstruction.version; builderPolicyId = WalkerBuilder.policyId policy
                operation = mutation.Operation; mutationSelection = selection
                parentClearance = WalkerClearance.inspect parent decoded.Genome |> snd
                changed = mutation.Changes; footCount = Motion.footCount mutation.Program
                gates = gates.ToArray(); generated = generated.ToArray(); rejected = rejected.ToArray()
                counts = {| proposals = 1; realizationAttempts = attempts; rejected = rejected.Count
                            duplicates = attempts-rejected.Count-generated.Count; exported = generated.Count; dynamicTrials = 0 |}
            |}
            File.WriteAllText(Path.Combine(directory, "candidates.json"), Realization.json summary+"\n")
            trace "building" (if generated.Count=0 then "rejected" else "complete") 0 $"Finished {attempts} attempts; exported {generated.Count}; rejected {rejected.Count}."
            printfn "%s" (Realization.json summary)
            if generated.Count = 0 then 1 else 0
        | "propose" :: args ->
            let options = parseOptions (Set.ofList ["--pairs"; "--variant"; "--towing"; "--seed"; "--attempt"; "--parent"]) args
            let remote =
                match Map.tryFind "--variant" options with
                | Some "remote" -> true | Some "wired" -> false
                | _ -> invalidArg "--variant" "Expected wired or remote."
            let parent = Map.tryFind "--parent" options |> Option.map Export.readGenome
            let towing =
                match Map.tryFind "--towing" options with
                | Some "yes" -> true | Some "no" | None -> false
                | _ -> invalidArg "--towing" "Expected yes or no."
            let genome, operation, changed =
                Evolution.propose (integer "--pairs" 3 options) remote towing (integer "--seed" 0 options)
                    (integer "--attempt" 0 options) parent
            let errors =
                match Grammar.compile Walker genome with Ok _ -> [] | Error errors -> errors
            printfn "%s" (JsonSerializer.Serialize {|
                grammarVersion = Export.grammarVersion; genome = Export.genomeDto genome
                operation = operation; changed = changed; accepted = errors.IsEmpty; errors = errors
            |})
            0
        | "generate" :: args ->
            let options = parseOptions (Set.ofList ["--output"; "--genome"; "--kind"; "--pairs"; "--ground"; "--coupler"; "--rocker"; "--phase"; "--seed"]) args
            let kind = options |> Map.tryFind "--kind" |> Option.defaultValue "walker" |> ModelKind.parse
            let defaults = Genome.defaults
            let genome =
              if Map.containsKey "--genome" options then
                if options |> Map.exists (fun key _ -> not (Set.contains key (Set.ofList ["--output"; "--genome"; "--kind"]))) then
                    invalidArg "--genome" "Do not combine --genome with individual genes."
                Export.readGenome options["--genome"]
              else
                { defaults with
                    Pairs = integer "--pairs" defaults.Pairs options
                    GroundHoles = integer "--ground" defaults.GroundHoles options
                    CouplerHoles = integer "--coupler" defaults.CouplerHoles options
                    RockerHoles = integer "--rocker" defaults.RockerHoles options
                    PhaseQuarterTurns = integer "--phase" defaults.PhaseQuarterTurns options
                    Seed = integer "--seed" defaults.Seed options }
            let assembly = compile kind genome
            let directory = Export.write (output options) assembly
            printfn "%s: %d parts, %d bodies, %d joints, %d gear meshes, %d motor(s)"
                directory assembly.Parts.Length
                (assembly.Parts |> List.choose _.Body |> Set.ofList |> Set.count)
                assembly.Joints.Length assembly.Gears.Length assembly.Drives.Length
            for warning in assembly.Warnings do eprintfn "NOTE: %s" warning
            0
        | "sweep" :: args ->
            let options = parseOptions (Set.ofList ["--output"; "--limit"; "--seed"]) args
            let limit = integer "--limit" 8 options
            if limit < 1 || limit > 32 then invalidArg "--limit" "limit must be between 1 and 32."
            let directory = output options
            if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries(directory) |> Seq.isEmpty) then
                invalidArg "--output" "Output directory must be empty."
            let candidates = Grammar.sweep (integer "--seed" 0 options)
            let valid = ResizeArray<_>()
            let rejected = ResizeArray<_>()
            for genome in candidates do
                match Grammar.compile Walker genome with
                | Error errors -> rejected.Add {| genome = genome; errors = errors |}
                | Ok assembly ->
                    if valid.Count < limit then
                        let name = $"candidate-{valid.Count:D3}"
                        Export.write (Path.Combine(directory, name)) assembly |> ignore
                        valid.Add {| directory = name; genome = genome |}
            if valid.Count = 0 then invalidOp "No candidate passed mechanical validation."
            let summary = {| schemaVersion = 1; seed = integer "--seed" 0 options
                             generated = valid.ToArray(); rejected = rejected.ToArray()
                             validButNotExported = candidates.Length - rejected.Count - valid.Count |}
            File.WriteAllText(Path.Combine(directory, "sweep.json"),
                JsonSerializer.Serialize(summary, JsonSerializerOptions(WriteIndented = true)) + "\n")
            printfn "Exported %d candidates; rejected %d invalid genomes. %s" valid.Count rejected.Count directory
            0
        | _ -> invalidArg "command" usage
    with
    | :? ArgumentException as e -> eprintfn "ERROR: %s" e.Message; 2
    | :? IOException as e -> eprintfn "ERROR: %s" e.Message; 2
    | :? UnauthorizedAccessException as e -> eprintfn "ERROR: %s" e.Message; 2
    | :? InvalidOperationException as e -> eprintfn "ERROR: %s" e.Message; 2
    | :? JsonException as e -> eprintfn "ERROR: %s" e.Message; 2
