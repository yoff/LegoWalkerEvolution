#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open LegoWalkers

let parentPath,output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--parent";parent;"--output";output] -> Path.GetFullPath parent,Path.GetFullPath output
    | _ -> invalidArg "arguments" "Expected --parent NATIVE_CANDIDATE --output EMPTY_DIRECTORY."
if output=parentPath || output.StartsWith(parentPath+string Path.DirectorySeparatorChar) then
    invalidArg "output" "Witness output must stay outside the immutable parent."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Witness output must be empty."
let unwrap result = result |> Result.defaultWith (Realization.json >> invalidOp)
let program = MotionExport.readProgramDocument (Path.Combine(parentPath,"program.json"))
let parent = Realization.screen program 72 |> unwrap
let genome = WalkerRealization.readGenome (Path.Combine(parentPath,"realization.json"))
let robot = WalkerRealization.compile parent genome |> unwrap
let attempts = WalkerLevelFeet.attemptsWithLimit 1 parent robot.Genome
let evidence = attempts |> List.map (fun (foot,anchor,toe,branch,side,result) ->
    {|foot=foot;anchor=anchor;toe=toe;branch=branch;side=side
      outcome=match result with Ok _ -> "constructed" | Error _ -> "rejected"
      diagnostic=match result with Ok _ -> "" | Error error -> Realization.json error|})
Directory.CreateDirectory output |> ignore
File.WriteAllText(Path.Combine(output,"attempts.json"),Realization.json {|
    schemaVersion=1;sourceParent=parentPath;parentGenotypeId=robot.GenotypeId
    operation="replace-double-parallelogram-foot";attempts=evidence;dynamicTrials=0|})
printfn "%d construction attempts; %d successful." evidence.Length (evidence |> List.filter (fun e -> e.outcome="constructed") |> List.length)
for code,items in evidence |> List.filter (fun e -> e.outcome="rejected") |> List.groupBy _.diagnostic do
    printfn "%d x %s" items.Length code
let generated = [
    for index,edit in attempts |> List.choose (fun (_,_,_,_,_,result) -> match result with Ok e -> Some e | Error _ -> None)
                      |> List.distinct |> List.groupBy (fun e -> e.Changes.Head |> fun (path,_,_) -> path)
                      |> List.map (fun (_,edits) -> edits.Head) |> List.indexed do
        let child = Realization.screen edit.Program 72 |> unwrap
        let realized = WalkerRealization.compile child edit.Genome |> unwrap
        let name = sprintf "case-%03d" (index+1)
        let batch = Path.Combine(output,name)
        Directory.CreateDirectory batch |> ignore
        let directory = Path.Combine(batch,"candidate-001")
        WalkerRealization.write directory child realized |> ignore
        File.WriteAllText(Path.Combine(batch,"candidates.json"),Realization.json {|
            schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=child.MechanismId
            operation=edit.Operation;parentGenotypeId=robot.GenotypeId;mutationVersion=WalkerEvolution.version
            gates=child.Decisions;changed=edit.Changes
            generated=[{|directory="candidate-001";genotypeId=realized.GenotypeId;assemblyId=realized.AssemblyId|}]
            rejected=([||]:obj array)
            counts={|proposals=1;realizationAttempts=1;rejected=0;duplicates=0;exported=1;dynamicTrials=0|}|})
        printfn "Exported %s: %d bodies." name child.Program.Bodies.Length
        yield {|directory=name;genotypeId=realized.GenotypeId;assemblyId=realized.AssemblyId|}
]
File.WriteAllText(Path.Combine(output,"witness.json"),Realization.json {|
    schemaVersion=1;gateVersion=Realization.gateVersion;footCount=Motion.footCount program
    operation="replace-double-parallelogram-foot";parentGenotypeId=robot.GenotypeId
    mutationVersion=WalkerEvolution.version;generated=generated;dynamicTrials=0|})
