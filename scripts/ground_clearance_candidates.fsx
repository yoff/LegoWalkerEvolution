#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open LegoWalkers

let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
let source, output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--parent"; source; "--output"; output] -> Path.GetFullPath source, Path.GetFullPath output
    | _ -> invalidArg "arguments" "Expected --parent NATIVE_DIRECTORY --output EMPTY_DIRECTORY."

let relative = Path.GetRelativePath(source, output)
if relative <> ".." && not (relative.StartsWith(".." + string Path.DirectorySeparatorChar)) && not (Path.IsPathRooted relative) then
    invalidArg "output" "Experiment output must be outside its immutable parent."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Experiment output must be empty."

let program = MotionExport.readProgramDocument (Path.Combine(source, "program.json"))
let genome = WalkerRealization.readGenome (Path.Combine(source, "realization.json"))
let parent = Realization.screen program 72 |> unwrap
let baseline = WalkerRealization.compile parent genome |> unwrap
let edits = WalkerStructural.footEdits parent baseline.Genome
let rejected = ResizeArray<_>()
let candidates = [
    for edit in edits do
        match Realization.screen edit.Program 72 with
        | Error gate -> rejected.Add {| operation = edit.Operation; decision = gate |}
        | Ok child ->
            match WalkerRealization.compile child { edit.Genome with MechanismId = child.MechanismId } with
            | Error gate -> rejected.Add {| operation = edit.Operation; decision = gate |}
            | Ok robot ->
                let geometry = Realization.screenAssemblyGeometry child robot.Assembly
                if geometry.Outcome = "reject" then
                    rejected.Add {| operation = edit.Operation; decision = geometry |}
                else yield edit,child,robot,geometry
]

Directory.CreateDirectory output |> ignore
let generated = ResizeArray<_>()
let write operation changes (child: ScreenedMotion) (robot: RealizedWalker) overlapCount =
    let name = $"case-{generated.Count+1:D3}"
    let directory = Path.Combine(output, name)
    WalkerRealization.write (Path.Combine(directory, "candidate-001")) child robot |> ignore
    let batch = {|
        schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = child.MechanismId
        gates = child.Decisions
        generated = [{| directory = "candidate-001"; assemblyId = robot.AssemblyId; genotypeId = robot.GenotypeId |}]
        rejected = ([]: GateDecision list)
        counts = {| proposals = 1; realizationAttempts = 1; rejected = 0; duplicates = 0; exported = 1; dynamicTrials = 0 |}
    |}
    File.WriteAllText(Path.Combine(directory, "candidates.json"), Realization.json batch + "\n")
    generated.Add {| directory = name; operation = operation; changed = changes
                     mechanismId = child.MechanismId; genotypeId = robot.GenotypeId; assemblyId = robot.AssemblyId
                     sampledEnvelopeOverlapPairs = overlapCount |}

let geometry = Realization.screenAssemblyGeometry parent baseline.Assembly
write "inherited-control" [] parent baseline geometry.Diagnostics.Length
for operation in ["lengthen-foot"; "bend-foot"] do
    let selected =
        candidates |> List.filter (fun (edit,_,_,_) -> edit.Operation = operation)
        |> List.distinctBy (fun (_,_,robot,_) -> robot.GenotypeId)
        |> List.sortBy (fun (_,_,robot,geometry) -> geometry.Diagnostics.Length,robot.GenotypeId)
        |> List.truncate 2
    if selected.IsEmpty then invalidOp $"No compiler-passing {operation} variant exists."
    for edit,child,robot,geometry in selected do
        write operation edit.Changes child robot geometry.Diagnostics.Length

let summary = {|
    schemaVersion = 1; gateVersion = Realization.gateVersion
    mutationVersion = WalkerEvolution.version; constructionVersion = WalkerConstruction.version
    parentGenotypeId = baseline.GenotypeId; parentAssemblyId = baseline.AssemblyId
    experiment = "ground-clearance-single-foot-v1"
    selection = "Two per family: fewest sampled envelope overlap pairs, then canonical genotype ID. Envelope warnings rank, never replace exact collision checks."
    limitations = ["Each variant edits one foot; coordinated changes to all feet are not tested."
                   "The inherited observation may depend on forbidden support; clearances are not proof of walking."]
    generated = generated.ToArray(); rejected = rejected.ToArray()
    counts = {| enumeratedEdits = edits.Length; compilerAttempts = edits.Length + 1; compilerPassing = candidates.Length
                exported = generated.Count; maxNativeLoadOnlyChecks = generated.Count; dynamicTrials = 0 |}
|}
File.WriteAllText(Path.Combine(output, "experiment.json"), Realization.json summary + "\n")
File.WriteAllText(Path.Combine(output, "seeds.json"),
                  Realization.json {| schemaVersion = 1; batches = generated |> Seq.map _.directory |> Seq.toList |} + "\n")
printfn "%s" (Realization.json summary.counts)
