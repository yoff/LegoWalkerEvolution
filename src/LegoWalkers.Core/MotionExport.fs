namespace LegoWalkers

open System
open System.IO
open System.Globalization
open System.Reflection
open System.Text.Json
open System.Text.Json.Serialization

module MotionExport =
    let private options =
        JsonSerializerOptions(
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
            WriteIndented = true)

    let programJson (program: MotionProgram) =
        JsonSerializer.Serialize({| schemaVersion = 1; program = program |}, options) + "\n"

    let readProgramDocument path =
        use doc = JsonDocument.Parse(File.ReadAllText path)
        let fields context (expected: string list) (value: JsonElement) =
            if value.ValueKind <> JsonValueKind.Object then invalidArg "program" $"{context} must be an object."
            let names = value.EnumerateObject() |> Seq.map _.Name |> Seq.toList
            if names.Length <> expected.Length || Set.ofList names <> Set.ofList expected then
                let required = String.concat ", " expected
                invalidArg "program" $"{context} requires exactly: {required}."
        let array context (value: JsonElement) =
            if value.ValueKind <> JsonValueKind.Array then invalidArg "program" $"{context} must be an array."
            value.EnumerateArray() |> Seq.toList
        let vector value = fields "vector" ["x"; "y"; "z"] value
        let port value = fields "port" ["body"; "site"] value
        fields "motion document" ["schemaVersion"; "program"] doc.RootElement
        if doc.RootElement.GetProperty("schemaVersion").GetInt32() <> 1 then
            invalidArg "program" "Unsupported motion program schema version."
        let program = doc.RootElement.GetProperty("program")
        fields "program" ["bodies"; "hinges"; "motor"; "transmissions"; "feet"] program
        for body in array "bodies" (program.GetProperty("bodies")) do
            fields "body" ["id"; "grounded"; "initial"; "sites"] body
            let pose = body.GetProperty("initial")
            fields "initial pose" ["origin"; "angle"] pose
            vector (pose.GetProperty("origin"))
            for site in array "sites" (body.GetProperty("sites")) do
                fields "site" ["id"; "position"] site
                vector (site.GetProperty("position"))
        for hinge in array "hinges" (program.GetProperty("hinges")) do
            fields "hinge" ["id"; "a"; "b"] hinge
            port (hinge.GetProperty("a"))
            port (hinge.GetProperty("b"))
        fields "motor" ["body"; "phase"] (program.GetProperty("motor"))
        for transmission in array "transmissions" (program.GetProperty("transmissions")) do
            fields "transmission" ["input"; "output"; "ratio"; "phase"] transmission
        for foot in array "feet" (program.GetProperty("feet")) do
            fields "foot" ["id"; "port"; "radius"] foot
            port (foot.GetProperty("port"))
        program.Deserialize<MotionProgram>(options)

    let readProgram path =
        let value = readProgramDocument path
        match Motion.compile value with
        | Ok _ -> value
        | Error errors -> invalidArg "program" (String.concat "; " errors)

    let private trajectory program graph frames =
        let vector (p: Vec3) = [|p.X; p.Y; p.Z|]
        let samples =
            [|for frame in frames ->
                {|
                    inputAngle = frame.InputAngle
                    maxResidualLdu = frame.MaxResidual
                    poses =
                        [|for KeyValue(id, pose) in frame.Poses ->
                            {| body = id; origin = vector pose.Origin; angle = pose.Angle |}|]
                    feet =
                        [|for KeyValue(id, position) in Motion.footPositions graph frame ->
                            {| id = id; position = vector position |}|]
                |}|]
        JsonSerializer.Serialize({|
            schemaVersion = 1
            units = "ldu"
            mode = "planar-kinematics-not-dynamics"
            program = program
            frames = samples
        |}, options)

    let cycleSvg (program: MotionProgram) graph (frames: MotionFrame list) =
        if frames.Length < 2 then invalidArg "frames" "A cycle visualization requires at least two frames."
        let number (value: float) = value.ToString("0.###", CultureInfo.InvariantCulture)
        let escaped (value: string) = System.Net.WebUtility.HtmlEncode value
        let world (frame: MotionFrame) (body: MotionBody) (site: MotionSite) =
            Motion.transform frame.Poses[body.Id] site.Position
        let points =
            [for frame in frames do
                for body in program.Bodies do
                    for site in body.Sites -> world frame body site]
        let left, right = (points |> List.minBy _.X).X, (points |> List.maxBy _.X).X
        let top, bottom = (points |> List.minBy _.Y).Y, (points |> List.maxBy _.Y).Y
        let scale = min (250.0/max 1.0 (right-left)) (280.0/max 1.0 (bottom-top))
        let project panel (point: Vec3) =
            let x = 25.0 + float panel*300.0 + (point.X-left)*scale
            let y = 85.0 + (point.Y-top)*scale
            $"{number x},{number y}"
        let colors = [|"#63758b"; "#b97800"; "#c74342"; "#28744b"; "#8454bb"; "#b36120"; "#197f8b"; "#786457"|]
        let svg = ResizeArray<string>()
        svg.Add """<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="440" viewBox="0 0 1200 440" style="max-width:100%;height:auto" role="img">"""
        svg.Add "<title>Motion grammar regression: four motor phases and solved foot trajectories</title>"
        svg.Add "<desc>Planar constraint kinematics, not a LEGO build or walking qualification. Dashed grey lines are grounded bodies; blue curves are foot paths.</desc>"
        svg.Add """<rect width="1200" height="440" fill="#f5f8fc"/><g font-family="system-ui,sans-serif" fill="#203147">"""
        svg.Add """<text x="24" y="28" font-size="19" font-weight="bold">One motor input / generic rigid-body and hinge constraints</text>"""
        for panel in 0..3 do
            let target = frames.Head.InputAngle + float panel*Math.PI/2.0
            let frame = frames |> List.minBy (fun frame -> abs(frame.InputAngle-target))
            svg.Add $"""<text x="{24+panel*300}" y="59" font-size="16">Motor {panel*90} degrees</text>"""
            for foot in program.Feet do
                let trace =
                    frames |> List.map (fun frame ->
                        (Motion.footPositions graph frame)[foot.Id] |> project panel) |> String.concat " "
                svg.Add $"""<polyline points="{trace}" fill="none" stroke="#3186cc" stroke-width="1.5" opacity="0.65"/>"""
            for index, body in program.Bodies |> List.indexed do
                let values = body.Sites |> List.map (world frame body)
                let color = colors[index%colors.Length]
                // A star keeps off-axis sites on a multi-site rigid body visibly connected.
                for other in values.Tail do
                    let dash = if body.Grounded then """ stroke-dasharray="4 4" """ else ""
                    svg.Add $"""<polyline points="{project panel values.Head} {project panel other}" fill="none" stroke="{color}" stroke-width="3" {dash}/>"""
                if values.Length = 3 then
                    let dash = if body.Grounded then """ stroke-dasharray="4 4" """ else ""
                    svg.Add $"""<polyline points="{project panel values[1]} {project panel values[2]}" fill="none" stroke="{color}" stroke-width="3" {dash}/>"""
                for point in values do
                    let coordinates = (project panel point).Split(',')
                    svg.Add $"""<circle cx="{coordinates[0]}" cy="{coordinates[1]}" r="3" fill="{color}"/>"""
            for foot in program.Feet do
                let p = (Motion.footPositions graph frame)[foot.Id]
                let coordinates = (project panel p).Split(',')
                svg.Add $"""<circle cx="{coordinates[0]}" cy="{coordinates[1]}" r="5" fill="#1876c1" stroke="white"/>"""
        svg.Add """<text x="24" y="395" font-size="13">Blue: complete foot path. Dots: attachment sites. Grey dashed: ground. Dimensions: LDU (+Y down).</text>"""
        let labels = program.Bodies |> List.map _.Id |> String.concat " / " |> escaped
        svg.Add $"""<text x="24" y="420" font-size="12">Bodies: {labels}</text></g></svg>"""
        String.concat "\n" svg + "\n"

    let writeTurns directory program steps turns =
        if steps < 36 || steps > 3600 then invalidArg "steps" "Use 36..3600 samples per motor revolution."
        if turns < 1 || turns > 32 then invalidArg "turns" "Use one to 32 motor revolutions."
        let graph =
            match Motion.compile program with
            | Ok graph -> graph
            | Error errors -> invalidArg "program" (String.concat "; " errors)
        let frames =
            match Motion.sample graph [for n in 0..steps*turns -> float n*2.0*Math.PI/float steps] with
            | Ok frames -> frames
            | Error error -> invalidArg "program" error
        let path = Path.GetFullPath directory
        if Directory.Exists path && not (Directory.EnumerateFileSystemEntries(path) |> Seq.isEmpty) then
            invalidArg "directory" "Output directory must be empty."
        let json = trajectory program graph frames
        use stream = Assembly.GetExecutingAssembly().GetManifestResourceStream("LegoWalkers.Core.MotionPreview.html")
        if isNull stream then invalidOp "Missing embedded motion preview template."
        use reader = new StreamReader(stream)
        let html = reader.ReadToEnd().Replace("__MOTION_JSON__", json)
        Directory.CreateDirectory(path) |> ignore
        File.WriteAllText(Path.Combine(path, "program.json"), programJson program)
        File.WriteAllText(Path.Combine(path, "motion.json"), json + "\n")
        File.WriteAllText(Path.Combine(path, "preview.html"), html)
        File.WriteAllText(Path.Combine(path, "cycle.svg"), cycleSvg program graph frames)
        path, frames.Length

    let write directory program steps = writeTurns directory program steps 1

    let writeTestGallery directory =
        let fourBar = MotionExamples.named "four-bar" |> Result.defaultWith invalidOp
        let dyad =
            Motion.addDyad "branch" (Motion.port "ground" "drive") (Motion.port "coupler" "foot") 180 180 1 fourBar
            |> Result.defaultWith invalidOp
        let cases = [
            "four-bar", fourBar
            "walker1", MotionExamples.walker1
            "added-dyad", { dyad with Feet = [Motion.foot "new-tip" "branch-a" "tip" 4] }
        ]
        let rendered =
            [for name, program in cases do
                let graph = Motion.compile program |> Result.defaultWith (String.concat "; " >> invalidOp)
                let frames =
                    Motion.sample graph [for n in 0..360 -> float n*Math.PI/180.0]
                    |> Result.defaultWith invalidOp
                yield name+".svg", cycleSvg program graph frames]
        Directory.CreateDirectory directory |> ignore
        for name, svg in rendered do File.WriteAllText(Path.Combine(directory, name), svg)
        rendered |> List.map fst
