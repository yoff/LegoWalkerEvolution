#load "PackagingRealizationSupport.fsx"

open System
open System.IO
open System.Text.Json
open LegoWalkers
open PackagingRealizationSupport

type Checkpoint = {
    Version: string
    Request: WalkerPackagingRequest
    PlanId: string
    DriveId: string
    ReferenceSha256: string
    SourceSha256: string
    OracleScope: string
    State: ConstructionSearchState
    PendingAssemblyId: string
    PendingGenotypeId: string
}

let run output (checkpoint: Checkpoint) =
    let state,witness =
        if checkpoint.State.Status="oracle-accepted" then checkpoint.State,None
        else WalkerConstructionTheory.advance checkpoint.Request checkpoint.OracleScope checkpoint.State
    let attempts = witness |> Option.toList |> List.map (fun construction ->
        {Orientation=0;Construction=Some construction;Gates=[]})
    emit output checkpoint.PlanId checkpoint.DriveId checkpoint.ReferenceSha256 checkpoint.SourceSha256 checkpoint.Request attempts
    let assemblyId,genotypeId =
        match witness,state.Pending with
        | Some construction,Some proof ->
            File.WriteAllText(Path.Combine(output,"candidate-01","construction-proof.json"),Realization.json proof+"\n")
            construction.Realized.AssemblyId,construction.Realized.GenotypeId
        | None,None -> "",""
        | _ -> invalidOp "Search witness and pending oracle proof are inconsistent."
    let saved = {checkpoint with State=state;PendingAssemblyId=assemblyId;PendingGenotypeId=genotypeId}
    File.WriteAllText(Path.Combine(output,"search-checkpoint.json"),Realization.json saved+"\n")
    printfn "%s" (Realization.json {|status=state.Status;visited=state.Visited;pruned=state.Pruned;discarded=state.Discarded|})

let start input plans plan output oracleScope states frontier motorDepthTolerance reduction =
    let request,drive,referenceSha,sourceSha = readRequest input plans plan output
    if not(Double.IsFinite motorDepthTolerance) || motorDepthTolerance<0.0 || motorDepthTolerance>120.0 then
        invalidArg "motor-depth-tolerance" "Qualification motor depth tolerance must be within 0..120 LDU."
    let targets = request.Targets |> List.map (fun target ->
        if target.Placement.Id=request.Source.MotorPart then
            {target with ToleranceLdu={target.ToleranceLdu with Z=max target.ToleranceLdu.Z motorDepthTolerance}}
        else target)
    let request = WalkerConstructionTheory.withMotorReduction reduction {request with Targets=targets}
    let checkpoint =
        {Version=WalkerConstructionTheory.version;Request=request;PlanId=plan;DriveId=drive
         ReferenceSha256=referenceSha;SourceSha256=sourceSha;OracleScope=oracleScope
         State=WalkerConstructionTheory.start request oracleScope states frontier;PendingAssemblyId="";PendingGenotypeId=""}
    run output checkpoint

let resume path verdictPath output oracleScope =
    use doc = JsonDocument.Parse(File.ReadAllText path)
    let checkpoint = parse<Checkpoint> doc.RootElement
    if checkpoint.Version<>WalkerConstructionTheory.version || checkpoint.OracleScope<>oracleScope
       || checkpoint.State.Scope<>WalkerConstructionTheory.scope checkpoint.Request oracleScope then
        invalidArg "checkpoint" "Construction theory, frozen request or oracle scope changed."
    if checkpoint.State.Pending.IsNone then invalidArg "checkpoint" "No candidate is awaiting feedback."
    let replay = WalkerConstructionTheory.expansion checkpoint.Request oracleScope
    match replay checkpoint.State.Pending.Value checkpoint.State.Nogoods with
    | Built(_,construction) when construction.Realized.AssemblyId=checkpoint.PendingAssemblyId
                                 && construction.Realized.GenotypeId=checkpoint.PendingGenotypeId -> ()
    | _ -> invalidArg "checkpoint" "Pending proof does not reconstruct its bound physical witness."
    use verdictDoc = JsonDocument.Parse(File.ReadAllText verdictPath)
    let verdict = verdictDoc.RootElement
    if text "scope" verdict<>checkpoint.State.Scope || text "assemblyId" verdict<>checkpoint.PendingAssemblyId
       || text "genotypeId" verdict<>checkpoint.PendingGenotypeId then
        invalidArg "verdict" "Oracle verdict belongs to a different construction witness."
    let evidencePath = text "evidencePath" verdict
    requireHash evidencePath (text "evidenceSha256" verdict)
    use evidenceDoc = JsonDocument.Parse(File.ReadAllText evidencePath)
    let evidence = evidenceDoc.RootElement
    let state,accepted =
        match text "status" evidence with
        | "ready-for-dynamics" ->
            let connectivity = evidence.GetProperty "constructionConnectivity"
            if text "status" connectivity<>"pass" || text "scope" connectivity<>checkpoint.State.Scope then
                invalidArg "verdict" "Exact connectivity evidence must pass in this frozen proof scope."
            let verified = connectivity.GetProperty("verifiedGoals") |> parse<string list>
            let preservation = evidence.GetProperty "jointPreservation"
            if text "status" preservation<>"pass" || text "scope" preservation<>checkpoint.State.Scope
               || text "ldrawSha256" preservation<>text "ldrawSha256" connectivity
               || text "partsDependencySha256" preservation<>text "partsDependencySha256" connectivity then
                invalidArg "verdict" "Joint preservation must pass for the same frozen assembly and parts library."
            let joints = preservation.GetProperty("verifiedGoals") |> parse<string list>
            let state,proof = ConstructionSearch.acceptVerified (digest evidencePath) ("oracle"::(verified @ joints)) checkpoint.State
            state,Some proof
        | "rejected" ->
            let mutable hints = Unchecked.defaultof<JsonElement>
            let kind,parts,bodies =
                if evidence.TryGetProperty("repairHints",&hints) then
                    text "kind" hints,hints.GetProperty("parts") |> parse<string list>,hints.GetProperty("bodies") |> parse<string list>
                else "rejected-without-repair-hints",[],[]
            let feedback = {Scope=checkpoint.State.Scope;EvidenceSha256=digest evidencePath
                            Reason=kind;Parts=parts;Bodies=bodies;Exact=true}
            ConstructionSearch.reject feedback checkpoint.State,None
        | status -> invalidArg "verdict" $"Not an exact offline gate verdict: {status}."
    run output {checkpoint with State=state}
    accepted |> Option.iter (fun proof ->
        File.WriteAllText(Path.Combine(output,"accepted-proof.json"),Realization.json proof+"\n"))
    File.Copy(verdictPath,Path.Combine(output,"oracle-verdict-binding.json"))

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--input";input;"--plans";plans;"--plan";plan;"--oracle-scope";oracle;"--max-states";states;"--max-frontier";frontier;"--output";output] ->
    start input plans plan (Path.GetFullPath output) oracle (Int32.Parse states) (Int32.Parse frontier) 0.0 1
| ["--input";input;"--plans";plans;"--plan";plan;"--oracle-scope";oracle;"--max-states";states;"--max-frontier";frontier;"--motor-depth-tolerance";depth;"--output";output] ->
    start input plans plan (Path.GetFullPath output) oracle (Int32.Parse states) (Int32.Parse frontier) (Double.Parse(depth,Globalization.CultureInfo.InvariantCulture)) 1
| ["--input";input;"--plans";plans;"--plan";plan;"--oracle-scope";oracle;"--max-states";states;"--max-frontier";frontier;"--motor-reduction";reduction;"--output";output] ->
    start input plans plan (Path.GetFullPath output) oracle (Int32.Parse states) (Int32.Parse frontier) 0.0 (Int32.Parse reduction)
| ["--input";input;"--plans";plans;"--plan";plan;"--oracle-scope";oracle;"--max-states";states;"--max-frontier";frontier;"--motor-depth-tolerance";depth;"--motor-reduction";reduction;"--output";output] ->
    start input plans plan (Path.GetFullPath output) oracle (Int32.Parse states) (Int32.Parse frontier) (Double.Parse(depth,Globalization.CultureInfo.InvariantCulture)) (Int32.Parse reduction)
| ["--resume";checkpoint;"--verdict";verdict;"--oracle-scope";oracle;"--output";output] ->
    let output = Path.GetFullPath output
    for path in [checkpoint;verdict] do
        let parent = Path.GetDirectoryName(Path.GetFullPath path)
        if output=parent || output.StartsWith(parent+string Path.DirectorySeparatorChar,StringComparison.Ordinal)
           || parent.StartsWith(output+string Path.DirectorySeparatorChar,StringComparison.Ordinal) then
            invalidArg "output" "Continuation output must be outside immutable checkpoint/evidence directories."
    resume checkpoint verdict output oracle
| _ -> invalidArg "arguments" "Expected frozen --input/--plans/--plan/--oracle-scope/--max-states/--max-frontier, or --resume/--verdict/--oracle-scope, followed by --output FRESH_DIR."
