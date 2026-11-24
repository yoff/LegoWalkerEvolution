namespace LegoWalkers

open System
open System.Globalization
open System.IO
open System.Security.Cryptography
open System.Text
open System.Text.Json

module Export =
    let simulatorRevision = "1b70ff4b6c567b3c8541d5ef8ff4c56b4be24a5b"
    let grammarVersion = "technic-fourbar-v5"
    let genomeDto (g: Genome) = {|
        pairs = g.Pairs; groundHoles = g.GroundHoles; couplerHoles = g.CouplerHoles
        rockerHoles = g.RockerHoles; phaseQuarterTurns = g.PhaseQuarterTurns; seed = g.Seed
        remote = g.Remote; batterySlot = g.BatterySlot; batteryLevel = g.BatteryLevel
        receiverSlot = g.ReceiverSlot; receiverSide = g.ReceiverSide; motorDirection = g.MotorDirection
        towWagon = g.TowWagon; batteryOnWagon = g.BatteryOnWagon
        receiverOnWagon = g.ReceiverOnWagon; wagonAxleHole = g.WagonAxleHole
    |}
    let readGenome path =
        let json = File.ReadAllText path
        use doc = JsonDocument.Parse json
        let required = ["pairs"; "groundHoles"; "couplerHoles"; "rockerHoles"; "phaseQuarterTurns"; "seed"
                        "remote"; "batterySlot"; "batteryLevel"; "receiverSlot"; "receiverSide"; "motorDirection"
                        "towWagon"; "batteryOnWagon"; "receiverOnWagon"; "wagonAxleHole"] |> Set.ofList
        let actual = doc.RootElement.EnumerateObject() |> Seq.map _.Name |> Set.ofSeq
        if actual <> required then invalidArg "genome" "Expected a complete v5 genome with no unknown fields."
        let genome = JsonSerializer.Deserialize<Genome>(json, JsonSerializerOptions(PropertyNameCaseInsensitive=true))
        match Genome.validate genome with
        | [] -> genome
        | errors -> invalidArg "genome" (String.concat "; " errors)
    let private number (n: float) =
        (if abs n < 5e-10 then 0.0 else n).ToString("0.#########", CultureInfo.InvariantCulture)
    let private vector (v: Vec3) = [| v.X; v.Y; v.Z |]
    let private matrix r = [|
        [|r.X.X; r.Y.X; r.Z.X|]
        [|r.X.Y; r.Y.Y; r.Z.Y|]
        [|r.X.Z; r.Y.Z; r.Z.Z|]
    |]
    let ldraw a =
        [
            $"0 LegoWalkers {ModelKind.name a.Kind}"
            $"0 // Grammar: {grammarVersion}; seed: {a.Genome.Seed}"
            "0 // Coordinates: LDU, +Y down; generated parts require a local LDraw library."
            for p in a.Parts do
                $"0 // instance: {p.Id}"
                let values =
                    [p.Position.X; p.Position.Y; p.Position.Z
                     p.Rotation.X.X; p.Rotation.Y.X; p.Rotation.Z.X
                     p.Rotation.X.Y; p.Rotation.Y.Y; p.Rotation.Z.Y
                     p.Rotation.X.Z; p.Rotation.Y.Z; p.Rotation.Z.Z]
                    |> List.map number |> String.concat " "
                $"1 {p.Color} {values} {p.Part}"
        ] |> String.concat "\r\n" |> fun s -> s + "\r\n"
    let sha256 (content: string) =
        SHA256.HashData(Encoding.UTF8.GetBytes content) |> Convert.ToHexString |> _.ToLowerInvariant()

    let assemblyId assembly =
        ldraw assembly |> _.Split('\n') |> Array.filter (fun line -> line.StartsWith "1 ")
        |> Array.map _.Trim() |> Array.sort |> String.concat "\n" |> sha256

    let manifest modelFile (a: Assembly) =
        let bodyIds = a.Parts |> List.choose _.Body |> List.distinct |> List.sort
        let dto = {|
            schemaVersion = 1
            grammarVersion = grammarVersion
            modelKind = ModelKind.name a.Kind
            modelFile = modelFile
            ldrawSha256 = sha256 (ldraw a)
            simulator = {| repository = "https://github.com/yoff/LegoTechnicSimulation"; revision = simulatorRevision |}
            units = {| length = "ldu"; lduToMeters = 0.0004; up = [|0; -1; 0|]; forward = [|1; 0; 0|] |}
            genome = genomeDto a.Genome
            payloads = [|
                if a.Genome.Remote then
                    {| partInstance = "payload-battery"; product = "8881"; carrier = if a.Genome.BatteryOnWagon then "wagon" else "walker" |}
                    {| partInstance = "payload-receiver"; product = "8884"; carrier = if a.Genome.ReceiverOnWagon then "wagon" else "walker" |}
            |]
            groundWheels = [|
                if a.Genome.TowWagon then
                    yield "wagon-wheel-0"
                    yield "wagon-wheel-1"
            |]
            chassis = a.Chassis
            parts = a.Parts |> List.map (fun p -> {|
                id = p.Id
                part = p.Part
                color = p.Color
                position = vector p.Position
                rotation = matrix p.Rotation
                body = p.Body |> Option.toObj
            |})
            expected = {|
                bodies = bodyIds
                joints = a.Joints |> List.map (fun j -> {|
                    id = j.Id
                    bodyA = j.BodyA
                    bodyB = j.BodyB
                    kind = "revolute"
                    position = vector j.Position
                    axis = vector j.Axis
                    connector = j.Connector
                |})
                gears = [|for g in a.Gears -> {|bodyA = g.BodyA; bodyB = g.BodyB; ratio = g.Ratio|}|]
                drives = [|for d in a.Drives -> {|motorPart = d.MotorPart; joint = d.Joint; speed = d.Speed; maxTorque = d.MaxTorque|}|]
            |}
            feet = [|for f in a.Feet -> {|id = f.Id; body = f.Body; position = vector f.Position;
                                       partInstance = f.Part; contactRadiusMeters = 0.004 |}|]
            footPaths = [|for p in a.FootPaths -> {|id = p.Id; strideLdu = p.Stride; liftLdu = p.Lift; points = p.Points |> List.map vector|}|]
            warnings = a.Warnings
        |}
        JsonSerializer.Serialize(dto, JsonSerializerOptions(WriteIndented = true)) + "\n"

    let write directory a =
        let directory = Path.GetFullPath directory
        let model = "model.ldr"
        if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries(directory) |> Seq.isEmpty) then
            invalidArg (nameof directory) $"Output directory is not empty: {directory}"
        Directory.CreateDirectory directory |> ignore
        let encoding = UTF8Encoding(false)
        File.WriteAllText(Path.Combine(directory, model), ldraw a, encoding)
        File.WriteAllText(Path.Combine(directory, "manifest.json"), manifest model a, encoding)
        directory
