#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"

open System
open System.IO
open System.Security.Cryptography
open System.Text.Json
open LegoWalkers

let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase,
    UnmappedMemberHandling=System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
    RespectRequiredConstructorParameters=true)
let parse<'a> (element: JsonElement) = JsonSerializer.Deserialize<'a>(element.GetRawText(),options)
let text (name: string) (element: JsonElement) = element.GetProperty(name).GetString()
let digest path = File.ReadAllBytes path |> SHA256.HashData |> Convert.ToHexString |> _.ToLowerInvariant()
let requireHash path expected =
    if digest path<>expected then invalidArg "input" $"Immutable input changed: {path}"

let emit output planId driveId referenceSha sourceSha (request: WalkerPackagingRequest) (attempts: WalkerPackagingAttempt list) =
    if Directory.Exists output then invalidArg "output" "Use a fresh output directory."
    Directory.CreateDirectory output |> ignore
    File.WriteAllText(Path.Combine(output,"request.json"),Realization.json request+"\n")
    let generated,rejected = ResizeArray<_>(),ResizeArray<_>()
    for index,attempt in List.indexed attempts do
        match attempt.Construction with
        | None -> rejected.Add {|attempt=index+1;orientation=attempt.Orientation;gates=attempt.Gates|}
        | Some construction ->
            let directory = $"candidate-{index+1:D2}"
            let candidate = Path.Combine(output,directory)
            WalkerClearance.writeRealization candidate construction.Screened construction.Realized |> ignore
            File.WriteAllText(Path.Combine(candidate,"packaging-construction.json"),
                Realization.json {| version=WalkerPackaging.version;planId=planId;driveId=driveId;
                                    sourceProgramSha256=referenceSha;sourceRealizationSha256=sourceSha;
                                    addedPartIds=construction.AddedPartIds;retainedFramePartIds=construction.RetainedFramePartIds;
                                    removedPartIds=construction.RemovedPartIds;derivation=construction.Derivation;
                                    movingClearance=construction.Clearance;
                                    limitations=["Bounded realization retains structural ground anchors, rails and bridge plates; this is not arbitrary chassis synthesis."
                                                 "Covered stock-beam brackets are physical parts and pins, not unbuilt socket requests."
                                                 "Stock axles span modeled bearing/crank socket walls and motor socket centers without extending through its housing; retention and load capacity are not certified."
                                                 "Target tolerances constrain repairs; original requests are not silently discarded."
                                                 "All parts must pass exact reconstruction, mass and collision checks before admission."] |}+"\n")
            generated.Add {|directory=directory;assemblyId=construction.Realized.AssemblyId;genotypeId=construction.Realized.GenotypeId|}
    let mechanism = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> invalidOp) |> _.MechanismId
    let batch = {|schemaVersion=1;gateVersion=Realization.gateVersion;mechanismId=mechanism;gates=[]
                  generated=List.ofSeq generated;rejected=List.ofSeq rejected
                  counts={|proposals=1;realizationAttempts=attempts.Length;exported=generated.Count;rejected=rejected.Count
                           duplicates=0;dynamicTrials=0|}|}
    File.WriteAllText(Path.Combine(output,"candidates.json"),Realization.json batch+"\n")
    printfn "%s" (Realization.json batch.counts)

let readRequest (inputPath: string) (planPath: string) (planId: string) (output: string) =
    let output = Path.GetFullPath output
    if Directory.Exists output then invalidArg "output" "Use a fresh output directory."
    use inputDoc = JsonDocument.Parse(File.ReadAllText inputPath)
    use planDoc = JsonDocument.Parse(File.ReadAllText planPath)
    let input,report = inputDoc.RootElement,planDoc.RootElement
    requireHash inputPath (text "inputSha256" report)
    let parent = text "parent" input
    for path in [parent;Path.GetDirectoryName(Path.GetFullPath inputPath);Path.GetDirectoryName(Path.GetFullPath planPath)] do
        let path = Path.GetFullPath path
        if output=path || output.StartsWith(path+string Path.DirectorySeparatorChar,StringComparison.Ordinal)
           || path.StartsWith(output+string Path.DirectorySeparatorChar,StringComparison.Ordinal) then
            invalidArg "output" "Output must be separate from all immutable inputs."
    let masses = input.GetProperty "sourceMass"
    for file,key in ["program.json","programSha256";"realization.json","realizationSha256";"manifest.json","parentManifestSha256"] do
        requireHash (Path.Combine(parent,file)) (text key masses)
    let plan = report.GetProperty("plans").EnumerateArray() |> Seq.find (fun plan -> text "id" plan=planId)
    let driveId = text "driveId" plan
    let drive = input.GetProperty("drives").EnumerateArray() |> Seq.find (fun drive -> text "id" drive=driveId)
    let drivePath = text "driveProgramPath" drive
    requireHash drivePath (text "driveProgramSha256" drive)
    requireHash (text "ldrawPath" drive) (text "ldrawSha256" drive)
    let program = MotionExport.readProgramDocument drivePath
    let reference = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let source = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
    let massParts = drive.GetProperty("parts").EnumerateArray() |> Seq.map (fun p -> text "id" p,text "body" p) |> Map.ofSeq
    let placements = drive.GetProperty "placements" |> parse<WalkerPlacement list>
    let request = {
        Reference=reference;Program=program;Source=source
        Targets=plan.GetProperty("targets").EnumerateArray() |> Seq.map (fun target ->
            let id = text "id" target
            let part =
                (source.Bodies |> List.collect _.Parts) |> List.find (fun p -> p.Id=id)
            {Placement={part with Position=target.GetProperty("position") |> parse<Vec3>
                                  Rotation=target.GetProperty("rotation") |> parse<Rotation>}
             ToleranceLdu=target.GetProperty("toleranceLdu") |> parse<Vec3>}) |> Seq.toList
        Drive={Root=text "root" drive;MotorClockScale=drive.GetProperty("clockScale").GetDouble();InternalCompoundBearings=false
               Gears=placements |> List.filter (fun p -> (Catalog.gearTeeth p.Part).IsSome) |> List.map (fun p -> massParts[p.Id],p)
               Axles=placements |> List.filter (fun p -> (Catalog.get p.Part).Connector=Some Axle) |> List.map (fun p -> massParts[p.Id],p)}
        ObsoletePartIds=input.GetProperty("removedLegacyIds") |> parse<string list>
    }
    request,driveId,text "programSha256" masses,text "realizationSha256" masses

let generate inputPath planPath planId output limit =
    let request,driveId,referenceSha,sourceSha = readRequest inputPath planPath planId output
    emit output planId driveId referenceSha sourceSha request (WalkerPackaging.realize request limit)

let repair (parent: string) (feedbackPath: string) (output: string) limit =
    let parent,output = Path.GetFullPath parent,Path.GetFullPath output
    if output=parent || output.StartsWith(parent+string Path.DirectorySeparatorChar,StringComparison.Ordinal)
       || parent.StartsWith(output+string Path.DirectorySeparatorChar,StringComparison.Ordinal) then
        invalidArg "output" "Repair output must be outside its immutable parent."
    let requestPath = Path.Combine(Path.GetDirectoryName parent,"request.json")
    use doc = JsonDocument.Parse(File.ReadAllText requestPath)
    let request = parse<WalkerPackagingRequest> doc.RootElement
    let program = MotionExport.readProgramDocument (Path.Combine(parent,"program.json"))
    let screened = Realization.screen program 72 |> Result.defaultWith (Realization.json >> invalidOp)
    let expected = Realization.screen request.Program 72 |> Result.defaultWith (Realization.json >> invalidOp)
    if screened.Program<>expected.Program then invalidArg "parent" "Repair parent differs from the frozen packaging motion."
    let realized = WalkerRealization.readGenome (Path.Combine(parent,"realization.json"))
                   |> WalkerRealization.compile screened |> Result.defaultWith (Realization.json >> invalidOp)
    use metadataDoc = JsonDocument.Parse(File.ReadAllText(Path.Combine(parent,"packaging-construction.json")))
    let metadata = metadataDoc.RootElement
    let construction = {
        Screened=screened;Realized=realized
        AddedPartIds=metadata.GetProperty("addedPartIds") |> parse<string list>
        RetainedFramePartIds=metadata.GetProperty("retainedFramePartIds") |> parse<string list>
        RemovedPartIds=metadata.GetProperty("removedPartIds") |> parse<string list>
        Derivation=metadata.GetProperty("derivation") |> parse<string list>
        Clearance=metadata.GetProperty("movingClearance") |> parse<WalkerClearanceReport>
    }
    let feedback = WalkerConstruction.readFeedback feedbackPath
    let repairs = WalkerPackaging.repair request construction feedback limit
    let attempts =
        if repairs.IsEmpty then
            [{Orientation=0;Construction=None;Gates=[{Gate="packaging-realization";Outcome="reject";Code="repair-search-exhausted";
                                                     Diagnostics=["No supported repair preserves the target constraints within this bounded construction grammar; not an impossibility proof."]}]}]
        else repairs |> List.map (fun c -> {Orientation=0;Construction=Some c;Gates=[]})
    emit output (text "planId" metadata) (text "driveId" metadata)
         (text "sourceProgramSha256" metadata) (text "sourceRealizationSha256" metadata) request attempts
    File.Copy(feedbackPath,Path.Combine(output,"feedback.json"))
