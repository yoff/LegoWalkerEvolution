#r "../src/LegoWalkers.Core/bin/Debug/net10.0/LegoWalkers.Core.dll"
#load "PackagingSupport.fsx"

open System.IO
open System.Security.Cryptography
open System.Text.Json
open LegoWalkers
open PackagingSupport

let evaluate (parent: string) (massPath: string) (output: string) =
    if File.Exists output then invalidArg "output" "Balance evidence must be written to a fresh path."
    let digest path = File.ReadAllBytes path |> SHA256.HashData |> System.Convert.ToHexString |> _.ToLowerInvariant()
    let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase,
        UnmappedMemberHandling=System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
        RespectRequiredConstructorParameters=true)
    let input = JsonSerializer.Deserialize<MassInput>(File.ReadAllText massPath,options)
    for file,expected in ["manifest.json",input.ParentManifestSha256;"program.json",input.ProgramSha256;"realization.json",input.RealizationSha256] do
        if digest(Path.Combine(parent,file))<>expected then invalidArg "input" $"Constructed mass input differs from {file}."
    let requestPath = Path.Combine(Path.GetDirectoryName parent,"request.json")
    let request = JsonSerializer.Deserialize<WalkerPackagingRequest>(File.ReadAllText requestPath,options)
    let unwrap result = Result.defaultWith (Realization.json >> invalidOp) result
    let reference = Realization.screen request.Reference 72 |> unwrap
    let inherited = WalkerRealization.compile reference request.Source |> unwrap
    let originalFrames,coverage = WalkerClearance.inspect reference inherited.Genome
    let program = MotionExport.readProgramDocument(Path.Combine(parent,"program.json"))
    let screened = Realization.screen program 72 |> unwrap
    if screened.Program<>(Realization.screen request.Program 72 |> unwrap).Program then
        invalidArg "input" "Constructed motion differs from the packaging request."
    let realized = WalkerRealization.readGenome(Path.Combine(parent,"realization.json")) |> WalkerRealization.compile screened |> unwrap
    let frames = Motion.sample screened.Graph (originalFrames |> List.map (fun frame -> frame.InputAngle*request.Drive.MotorClockScale))
                 |> Result.defaultWith invalidOp
    let samples = PackagingMath.motionSamples screened realized frames 8.0
    let partIds = realized.Assembly.Parts |> List.map _.Id |> Set.ofList
    if input.Parts.Length<>partIds.Count || (input.Parts |> List.map _.Id |> Set.ofList)<>partIds then
        invalidArg "input" "Reconstructed mass must cover each constructed part exactly once."
    if input.Parts |> List.exists (fun p -> p.MassKg<=0.0 || not(System.Double.IsFinite p.MassKg)
                                          || not(Vec3.finite p.Center)) then
        invalidArg "input" "Each reconstructed part requires finite positive mass and a finite center."
    let total = input.Parts |> List.sumBy _.MassKg
    let phases = List.zip frames samples |> List.map (fun (frame,sample) ->
        let moment = input.Parts |> List.fold (fun sum part ->
            Vec3.add sum (Vec3.scale part.MassKg (PackagingMath.world screened frame part.Body part.Center))) Vec3.zero
        let center = Vec3.scale (1.0/total) moment
        {|referenceInputAngleRadians=frame.InputAngle/request.Drive.MotorClockScale;center=center;
          marginLdu=PackagingMath.margin sample.SupportHull center;heightLdu=sample.GroundY-center.Y|})
    let mean = PackagingMath.meanOverAngles (originalFrames |> List.map _.InputAngle |> List.toArray)
                                           (phases |> List.map _.heightLdu |> List.toArray)
    let report = {|schemaVersion=1;status="constructed-reconstructed-mass-balance-advisory";totalMassKg=total;
                   partCount=input.Parts.Length;meanHeightLdu=mean;worstMarginLdu=phases |> List.map _.marginLdu |> List.min;
                   outsideSupportSamples=phases |> List.filter (fun p -> p.marginLdu<0.0) |> List.length;
                   phases=phases;motionCoverage=coverage;physicsTrials=0;
                   limitations=["Mass includes every constructed frame, bracket, gear, axle and pin; electronics use the existing profile."
                                "Reference-phase weighted COM and assumed foot support are advisory; no contact forces or dynamic stability proof."]|}
    File.WriteAllText(output,Realization.json report+"\n")

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--parent";parent;"--mass-properties";mass;"--output";output] -> evaluate parent mass output
| _ -> invalidArg "arguments" "Expected --parent CANDIDATE --mass-properties RECONSTRUCTED_MASS --output FRESH_JSON."
