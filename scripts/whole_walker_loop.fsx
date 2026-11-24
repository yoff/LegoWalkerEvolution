#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open LegoWalkers

let parentPath,output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--parent";parent;"--output";output] -> Path.GetFullPath parent,Path.GetFullPath output
    | _ -> invalidArg "arguments" "Expected --parent NATIVE_CANDIDATE --output EMPTY_DIRECTORY."
if output = parentPath || output.StartsWith(parentPath+string Path.DirectorySeparatorChar) then
    invalidArg "output" "Witness output must stay outside the immutable parent."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Witness output must be empty."
let unwrap result = result |> Result.defaultWith (Realization.json >> invalidOp)
let program = MotionExport.readProgramDocument (Path.Combine(parentPath,"program.json"))
let parent = Realization.screen program 72 |> unwrap
let original = WalkerRealization.readGenome (Path.Combine(parentPath,"realization.json"))
let robot = WalkerRealization.compile parent original |> unwrap
let sample = WalkerEvolution.chooseWithSelection parent robot.Genome
let seen = Collections.Generic.HashSet<string>()
let attempts = ResizeArray<_>()
let mutable found = None
for seed in 0..255 do
    let mutation,selection = sample seed 0
    if found.IsNone && mutation.Operation = "add-hinged-foot"
       && seen.Add(Realization.json mutation.Changes) then
        let result =
            Realization.screen mutation.Program 72 |> Result.bind (fun child ->
                WalkerEvolution.realize parent robot.Genome mutation child 1 |> Result.bind (fun choices ->
                    WalkerRealization.compile child choices.Head |> Result.bind (fun offspring ->
                        let geometry = Realization.screenAssemblyGeometry child offspring.Assembly
                        if geometry.Outcome = "reject" then Error geometry else Ok (child,offspring))))
        match result with
        | Error gate -> attempts.Add {|seed=seed;outcome="rejected";diagnostic=Realization.json gate|}
        | Ok (child,offspring) ->
            attempts.Add {|seed=seed;outcome="exported";diagnostic=""|}
            found <- Some (seed,mutation,selection,child,offspring)
let seed,mutation,selection,child,offspring =
    found |> Option.defaultWith (fun () -> invalidOp $"No bounded whole-walker loop witness: {Realization.json attempts}")
Directory.CreateDirectory output |> ignore
WalkerBuilder.write output WalkerBuilder.defaultPolicy
WalkerRealization.write (Path.Combine(output,"parent")) parent robot |> ignore
WalkerRealization.write (Path.Combine(output,"candidate-001")) child offspring |> ignore
MotionExport.write (Path.Combine(output,"kinematics")) child.Program 72 |> ignore
let summary = {|
    schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=child.MechanismId
    mutationVersion=WalkerEvolution.version;parentGenotypeId=robot.GenotypeId;seed=seed;attempt=0
    constructionVersion=WalkerConstruction.version;builderPolicyId=WalkerBuilder.policyId WalkerBuilder.defaultPolicy
    operation=mutation.Operation;mutationSelection=selection;changed=mutation.Changes
    footCount=Motion.footCount child.Program;gates=child.Decisions
    generated=[{|directory="candidate-001";assemblyId=offspring.AssemblyId;genotypeId=offspring.GenotypeId|}]
    rejected=([||]:obj array)
    counts={|proposals=1;realizationAttempts=1;rejected=0;duplicates=0;exported=1;dynamicTrials=0|}
|}
File.WriteAllText(Path.Combine(output,"candidates.json"),Realization.json summary+"\n")
File.WriteAllText(Path.Combine(output,"witness.json"),Realization.json {|
    schemaVersion=1;classification="current-parent-whole-walker-mutation-witness"
    sourceParent=parentPath;parentGenotypeId=robot.GenotypeId;childGenotypeId=offspring.GenotypeId
    sourceFiles=["program.json";"realization.json";"manifest.json";"model.ldr"] |> List.map (fun name ->
        {|file=name;sha256=Export.sha256 (File.ReadAllText(Path.Combine(parentPath,name)))|})
    seed=seed;attempt=0;operation=mutation.Operation;changed=mutation.Changes
    parentBodies=program.Bodies.Length;childBodies=child.Program.Bodies.Length
    logicalFeet=Motion.footCount child.Program;dynamicTrials=0
    boundedSeedSearch=256;attempts=attempts
|}+"\n")
printfn "Seed %d, attempt 0: %s; %d -> %d bodies, %d logical feet; zero dynamics."
    seed mutation.Operation program.Bodies.Length child.Program.Bodies.Length (Motion.footCount child.Program)
