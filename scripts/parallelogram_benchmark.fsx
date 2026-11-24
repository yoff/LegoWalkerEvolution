#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open LegoWalkers

let output =
    match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
    | ["--output"; path] -> Path.GetFullPath path
    | _ -> invalidArg "arguments" "Expected --output EMPTY_DIRECTORY."
if Directory.Exists output && not (Directory.EnumerateFileSystemEntries output |> Seq.isEmpty) then
    invalidArg "output" "Benchmark output must be empty."
let unwrap result = result |> Result.defaultWith (Realization.json >> invalidOp)
Directory.CreateDirectory output |> ignore
let cases = [
    for name,toe in ["outward",40.0;"inward",-40.0] do
        let program = MotionExamples.doubleParallelogram 60 toe |> Result.defaultWith invalidOp
        let screened = Realization.screen program 144 |> unwrap
        let proposed = Realization.propose screened 2 |> unwrap
        let attempts = [
            for i,genome in proposed |> List.indexed do
                let realized = Realization.compile screened genome |> unwrap
                let directory = Path.Combine(name,$"side-{genome.Side}")
                Realization.write (Path.Combine(output,directory)) screened realized |> ignore
                yield {|directory=directory; side=genome.Side; assemblyId=realized.AssemblyId
                        genotypeId=realized.GenotypeId; parts=realized.Assembly.Parts.Length
                        bodies=program.Bodies.Length; hinges=program.Hinges.Length; proposal=i|}
        ]
        yield {|orientation=name; mechanismId=screened.MechanismId; constructions=attempts|}
]
File.WriteAllText(Path.Combine(output,"benchmark.json"),Realization.json {|
    schemaVersion=1; classification="native-construction-benchmark-not-campaign-founder"
    operations=["orientation-preserving-parallelogram";"hinged-foot"]
    cases=cases; dynamicTrials=0; liveMutationFamiliesChanged=false
|}+"\n")
printfn "Native compiler exported %d constructions across both L directions and depth sides; zero dynamics."
    (cases |> List.sumBy (fun c -> c.constructions.Length))
