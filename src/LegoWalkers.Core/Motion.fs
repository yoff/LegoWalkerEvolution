namespace LegoWalkers

open System

type MotionPose = { Origin: Vec3; Angle: float }
type MotionSite = { Id: string; Position: Vec3 }
type MotionBody = { Id: string; Grounded: bool; Initial: MotionPose; Sites: MotionSite list }
type MotionPort = { Body: string; Site: string }
type MotionHinge = { Id: string; A: MotionPort; B: MotionPort }
type MotionMotor = { Body: string; Phase: float }
type MotionTransmission = { Input: string; Output: string; Ratio: float; Phase: float }
type MotionFoot = { Id: string; Port: MotionPort; Radius: float }
type MotionProgram = {
    Bodies: MotionBody list
    Hinges: MotionHinge list
    Motor: MotionMotor
    Transmissions: MotionTransmission list
    Feet: MotionFoot list
}
type MotionFrame = { InputAngle: float; Poses: Map<string, MotionPose>; MaxResidual: float }

type CompiledMotion = private {
    Program: MotionProgram
    Moving: MotionBody array
    Offsets: Map<string, int>
    Bodies: Map<string, MotionBody>
    Sites: Map<MotionPort, Vec3>
}

module Motion =
    let maximumFeet = 6
    let footCount (program: MotionProgram) = program.Feet.Length
    let private planar p = Vec3.finite p && abs p.Z < 1e-10
    let private angleScale = 20.0
    let private tolerance = 1e-7

    let site id x y : MotionSite = { Id = id; Position = Vec3.create x y 0 }
    let port body site : MotionPort = { Body = body; Site = site }
    let rigid id x y angle sites : MotionBody =
        { Id = id; Grounded = false; Initial = { Origin = Vec3.create x y 0; Angle = angle }; Sites = sites }
    let ground id sites = { rigid id 0 0 0 sites with Grounded = true }
    let couple id a b : MotionHinge = { Id = id; A = a; B = b }
    let transmit input output ratio phase : MotionTransmission =
        { Input = input; Output = output; Ratio = ratio; Phase = phase }
    let foot id body site radius : MotionFoot = { Id = id; Port = port body site; Radius = radius }

    let transform (pose: MotionPose) (local: Vec3) =
        Vec3.add pose.Origin (Rotation.apply (Rotation.aroundZ pose.Angle) local)

    let compile (program: MotionProgram) =
        let bodyIds = program.Bodies |> List.map _.Id
        let bodies = program.Bodies |> List.map (fun b -> b.Id, b) |> Map.ofList
        let sites =
            program.Bodies
            |> List.collect (fun b -> b.Sites |> List.map (fun s -> port b.Id s.Id, s.Position))
            |> Map.ofList
        let validId = String.IsNullOrWhiteSpace >> not
        let unique (values: string list) = values.Length = (Set.ofList values).Count
        let errors = [
            if not (unique bodyIds) then "Body IDs must be unique."
            if program.Bodies.IsEmpty || not (program.Bodies |> List.exists _.Grounded) then
                "At least one grounded body is required."
            for body in program.Bodies do
                if not (validId body.Id) || not (planar body.Initial.Origin) || not (Double.IsFinite body.Initial.Angle) then
                    $"Body {body.Id} has an invalid ID or planar initial pose."
                if body.Sites.IsEmpty || not (unique (body.Sites |> List.map _.Id)) then
                    $"Body {body.Id} requires unique attachment sites."
                for site in body.Sites do
                    if not (validId site.Id) || not (planar site.Position) then $"Invalid site {body.Id}/{site.Id}."
            if not (unique (program.Hinges |> List.map _.Id)) then "Hinge IDs must be unique."
            for hinge in program.Hinges do
                if not (validId hinge.Id) || hinge.A.Body = hinge.B.Body then $"Invalid hinge {hinge.Id}."
                for endpoint in [hinge.A; hinge.B] do
                    if not (sites.ContainsKey endpoint) then $"Hinge {hinge.Id} references missing site {endpoint}."
            if not (Double.IsFinite program.Motor.Phase) then "Motor phase must be finite."
            match Map.tryFind program.Motor.Body bodies with
            | Some body when not body.Grounded -> ()
            | _ -> "The single motor must drive a moving body."
            for gear in program.Transmissions do
                if not (Double.IsFinite gear.Ratio && Double.IsFinite gear.Phase) || gear.Ratio = 0.0 then
                    "Transmission ratios must be finite and nonzero; phases must be finite."
                if gear.Input = gear.Output then "A transmission must connect different bodies."
                for endpoint in [gear.Input; gear.Output] do
                    match Map.tryFind endpoint bodies with
                    | Some body when not body.Grounded -> ()
                    | _ -> $"Transmission references missing or grounded body {endpoint}."
            if program.Feet.IsEmpty || not (unique (program.Feet |> List.map _.Id)) then
                "At least one uniquely named foot output is required."
            if footCount program > maximumFeet then $"A design may have at most {maximumFeet} logical feet."
            let footSites =
                program.Feet |> List.choose (fun f ->
                    Map.tryFind f.Port sites |> Option.map (fun p -> f.Port.Body, p))
            if footSites.Length <> (footSites |> List.distinct).Length then
                "Each logical foot must have a distinct output site; contact patches are not additional feet."
            for foot in program.Feet do
                if not (validId foot.Id) || not (sites.ContainsKey foot.Port) then $"Invalid foot output {foot.Id}."
                elif bodies[foot.Port.Body].Grounded then $"Foot {foot.Id} cannot be grounded."
                if not (Double.IsFinite foot.Radius) || foot.Radius <= 0.0 then $"Invalid foot radius {foot.Id}."
        ]
        if not errors.IsEmpty then Error errors
        else
            let mutable reached = program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
            let edges = program.Hinges |> List.map (fun h -> h.A.Body, h.B.Body)
            let mutable changed = true
            while changed do
                let before = reached.Count
                for a, b in edges do
                    if reached.Contains a || reached.Contains b then reached <- reached.Add(a).Add(b)
                changed <- before <> reached.Count
            if reached.Count <> program.Bodies.Length then Error ["Every body must be physically connected to ground by joints."]
            else
                let moving = program.Bodies |> List.filter (fun b -> not b.Grounded) |> List.toArray
                let equations = 2*program.Hinges.Length + 1 + program.Transmissions.Length
                if equations < 3*moving.Length then Error ["Mechanism is underconstrained after prescribing its single motor input."]
                else
                    Ok { Program = program; Moving = moving; Bodies = bodies; Sites = sites
                         Offsets = moving |> Array.mapi (fun i b -> b.Id, 3*i) |> Map.ofArray }

    let private poses graph (x: float array) =
        graph.Bodies |> Map.map (fun id body ->
            match Map.tryFind id graph.Offsets with
            | None -> body.Initial
            | Some i -> { Origin = Vec3.create x[i] x[i+1] 0; Angle = x[i+2] })

    let private equations graph input x =
        let current = poses graph x
        let rows = 2*graph.Program.Hinges.Length + 1 + graph.Program.Transmissions.Length
        let residual = Array.zeroCreate rows
        let jacobian = Array2D.zeroCreate rows x.Length
        let endpoint row sign (port: MotionPort) =
            let pose = current[port.Body]
            let local = graph.Sites[port]
            let offset = Rotation.apply (Rotation.aroundZ pose.Angle) local
            let world = Vec3.add pose.Origin offset
            residual[row] <- residual[row] + sign*world.X
            residual[row+1] <- residual[row+1] + sign*world.Y
            match Map.tryFind port.Body graph.Offsets with
            | Some i ->
                jacobian[row,i] <- jacobian[row,i] + sign
                jacobian[row+1,i+1] <- jacobian[row+1,i+1] + sign
                jacobian[row,i+2] <- jacobian[row,i+2] - sign*offset.Y
                jacobian[row+1,i+2] <- jacobian[row+1,i+2] + sign*offset.X
            | None -> ()
        for index, hinge in graph.Program.Hinges |> List.indexed do
            endpoint (2*index) 1.0 hinge.A
            endpoint (2*index) -1.0 hinge.B
        let motorRow = 2*graph.Program.Hinges.Length
        let motor = graph.Program.Motor
        residual[motorRow] <- angleScale*(current[motor.Body].Angle - input - motor.Phase)
        jacobian[motorRow,graph.Offsets[motor.Body]+2] <- angleScale
        for index, gear in graph.Program.Transmissions |> List.indexed do
            let row = motorRow + 1 + index
            residual[row] <- angleScale*(current[gear.Output].Angle - gear.Ratio*current[gear.Input].Angle - gear.Phase)
            jacobian[row,graph.Offsets[gear.Output]+2] <- angleScale
            jacobian[row,graph.Offsets[gear.Input]+2] <- -angleScale*gear.Ratio
        residual, jacobian

    let private rank (matrix: float[,]) =
        let a = Array2D.copy matrix
        let rows, columns = Array2D.length1 a, Array2D.length2 a
        let mutable row = 0
        for column in 0..columns-1 do
            if row < rows then
                let pivot = [row..rows-1] |> List.maxBy (fun i -> abs a[i,column])
                if abs a[pivot,column] > 1e-8 then
                    for j in column..columns-1 do
                        let value = a[row,j]
                        a[row,j] <- a[pivot,j]
                        a[pivot,j] <- value
                    for i in row+1..rows-1 do
                        let ratio = a[i,column]/a[row,column]
                        for j in column..columns-1 do a[i,j] <- a[i,j] - ratio*a[row,j]
                    row <- row+1
        row

    let private linearSolve (matrix: float[,]) (rhs: float array) =
        let n = rhs.Length
        let a, b = Array2D.copy matrix, Array.copy rhs
        let mutable valid = true
        for k in 0..n-1 do
            let pivot = [k..n-1] |> List.maxBy (fun i -> abs a[i,k])
            if abs a[pivot,k] < 1e-14 then valid <- false
            elif valid then
                for j in k..n-1 do
                    let value = a[k,j]
                    a[k,j] <- a[pivot,j]
                    a[pivot,j] <- value
                let value = b[k]
                b[k] <- b[pivot]
                b[pivot] <- value
                for i in k+1..n-1 do
                    let ratio = a[i,k]/a[k,k]
                    for j in k..n-1 do a[i,j] <- a[i,j] - ratio*a[k,j]
                    b[i] <- b[i] - ratio*b[k]
        if not valid then None
        else
            let solution = Array.zeroCreate n
            for i in n-1..-1..0 do
                let mutable value = b[i]
                for j in i+1..n-1 do value <- value - a[i,j]*solution[j]
                solution[i] <- value/a[i,i]
            if Array.forall Double.IsFinite solution then Some solution else None

    let private solveAt graph input (initial: float array) =
        let mutable x = Array.copy initial
        let mutable iteration = 0
        let mutable failure = None
        let mutable converged = false
        let norm r = r |> Array.sumBy (fun value -> value*value)
        while not converged && failure.IsNone && iteration < 80 do
            let residual, jacobian = equations graph input x
            let maximum = residual |> Array.map abs |> Array.max
            if maximum <= tolerance then
                if rank jacobian < x.Length then failure <- Some "Singular or underconstrained mechanism at this motor phase."
                else converged <- true
            else
                let n = x.Length
                let normal = Array2D.zeroCreate n n
                let rhs = Array.zeroCreate n
                for row in 0..residual.Length-1 do
                    for i in 0..n-1 do
                        rhs[i] <- rhs[i] - jacobian[row,i]*residual[row]
                        for j in 0..n-1 do normal[i,j] <- normal[i,j] + jacobian[row,i]*jacobian[row,j]
                for i in 0..n-1 do normal[i,i] <- normal[i,i] + 1e-9
                match linearSolve normal rhs with
                | None -> failure <- Some "Non-finite or unsolvable motion constraints."
                | Some step ->
                    let mutable accepted = false
                    let mutable fraction = 1.0
                    while not accepted && fraction >= 1.0/1024.0 do
                        let candidate = Array.map2 (fun value delta -> value+fraction*delta) x step
                        let next, _ = equations graph input candidate
                        if norm next < norm residual then
                            x <- candidate
                            accepted <- true
                        else fraction <- fraction/2.0
                    if not accepted then failure <- Some "Motion constraints cannot close on the selected assembly branch."
            iteration <- iteration+1
        if not converged then Error (failure |> Option.defaultValue "Motion constraints did not converge within 80 iterations.")
        else
            let residual, _ = equations graph input x
            Ok { InputAngle = input; Poses = poses graph x; MaxResidual = residual |> Array.map abs |> Array.max }

    let private coordinates graph (current: Map<string, MotionPose>) =
        graph.Moving |> Array.collect (fun body ->
            let pose = current[body.Id]
            [|pose.Origin.X; pose.Origin.Y; pose.Angle|])

    let solve graph angle previous =
        if not (Double.IsFinite angle) then Error "Motor angle must be finite."
        else
            let initial = graph.Bodies |> Map.map (fun _ body -> body.Initial)
            let startAngle, start =
                match previous with
                | Some frame -> frame.InputAngle, frame.Poses
                | None -> initial[graph.Program.Motor.Body].Angle - graph.Program.Motor.Phase, initial
            if (Set.ofSeq start.Keys <> Set.ofSeq graph.Bodies.Keys
                || start |> Map.exists (fun _ p -> not (planar p.Origin && Double.IsFinite p.Angle))
                || not (Double.IsFinite startAngle)) then Error "Previous frame does not match this mechanism."
            elif abs (angle-startAngle) > 8.0*Math.PI then Error "Sample a long motion in increments of at most four motor turns."
            else
                let count = max 1 (int (ceil (abs (angle-startAngle)/0.05)))
                let mutable frame = solveAt graph startAngle (coordinates graph start)
                let mutable index = 1
                while index <= count && Result.isOk frame do
                    match frame with
                    | Ok current ->
                        let input = startAngle + (angle-startAngle)*float index/float count
                        frame <- solveAt graph input (coordinates graph current.Poses)
                    | Error _ -> ()
                    index <- index+1
                frame

    let position graph frame port =
        match Map.tryFind port graph.Sites, Map.tryFind port.Body frame.Poses with
        | Some local, Some pose -> Ok (transform pose local)
        | _ -> Error $"Missing motion port {port}."

    let sample graph angles =
        let frames = ResizeArray<MotionFrame>()
        let mutable previous = None
        let mutable failure = None
        for angle in angles do
            if failure.IsNone then
                match solve graph angle previous with
                | Ok frame -> frames.Add frame; previous <- Some frame
                | Error message -> failure <- Some $"Motor angle {angle:G8}: {message}"
        match failure with Some error -> Error error | None -> Ok (List.ofSeq frames)

    let footPositions graph frame =
        graph.Program.Feet |> List.map (fun foot ->
            match position graph frame foot.Port with
            | Ok point -> foot.Id, point
            | Error error -> invalidOp error) |> Map.ofList

    let addDyad prefix a b lengthA lengthB branch (program: MotionProgram) =
        let bodyA, bodyB, middle = prefix+"-a", prefix+"-b", prefix+"-middle"
        if not (Double.IsFinite lengthA && Double.IsFinite lengthB) || lengthA <= 0.0 || lengthB <= 0.0 then
            Error "Dyad link lengths must be positive and finite."
        elif branch <> 1 && branch <> -1 then Error "Dyad assembly branch must be +1 or -1."
        else
            match compile program with
            | Error errors -> Error (String.concat "; " errors)
            | Ok graph ->
                let start = graph.Bodies[program.Motor.Body].Initial.Angle-program.Motor.Phase
                match solve graph start None with
                | Error error -> Error error
                | Ok frame ->
                    match position graph frame a, position graph frame b with
                    | Ok pa, Ok pb ->
                        let delta = Vec3.subtract pb pa
                        let distance = Vec3.length delta
                        if distance < 1e-9 || distance >= lengthA+lengthB-1e-6 || distance <= abs(lengthA-lengthB)+1e-6 then
                            Error "Dyad cannot close without a singularity at its assembly pose."
                        else
                            let u = Vec3.scale (1.0/distance) delta
                            let along = (lengthA*lengthA-lengthB*lengthB+distance*distance)/(2.0*distance)
                            let height = float branch*sqrt (lengthA*lengthA-along*along)
                            let joint = Vec3.add pa (Vec3.add (Vec3.scale along u) (Vec3.scale height (Vec3.create -u.Y u.X 0)))
                            let link id (origin: Vec3) length =
                                rigid id origin.X origin.Y (atan2 (joint.Y-origin.Y) (joint.X-origin.X))
                                    [site "root" 0 0; site "tip" length 0]
                            let initialized =
                                program.Bodies |> List.map (fun body -> { body with Initial = frame.Poses[body.Id] })
                            let candidate = { program with
                                                Bodies = initialized @ [link bodyA pa lengthA; link bodyB pb lengthB]
                                                Hinges = program.Hinges @ [
                                                    couple (prefix+"-root-a") a (port bodyA "root")
                                                    couple (prefix+"-root-b") b (port bodyB "root")
                                                    couple middle (port bodyA "tip") (port bodyB "tip")
                                                ] }
                            match compile candidate with
                            | Ok _ -> Ok candidate
                            | Error errors -> Error (String.concat "; " errors)
                    | Error error, _ | _, Error error -> Error error

    /// Complete a parallelogram around an existing hinged arm, adding two bodies without another input.
    let addParallelogram prefix ((a, b): MotionPort * MotionPort) ((root, tip): MotionPort * MotionPort)
                         (outputA, outputB) sites (program: MotionProgram) =
        let parallelId, outputId = prefix+"-parallel", prefix+"-output"
        if String.IsNullOrWhiteSpace prefix then Error "Parallelogram prefix must be nonempty."
        elif a.Body <> b.Body || root.Body <> tip.Body || a.Body = root.Body then
            Error "A parallelogram needs two reference sockets on one body and two driver sockets on another."
        else
            match compile program with
            | Error errors -> Error (String.concat "; " errors)
            | Ok graph ->
                let start = graph.Bodies[program.Motor.Body].Initial.Angle-program.Motor.Phase
                match solve graph start None with
                | Error error -> Error error
                | Ok frame ->
                    match position graph frame a, position graph frame b, position graph frame root, position graph frame tip with
                    | Ok pa, Ok pb, Ok pr, Ok pt ->
                        let reference, driver = Vec3.subtract pb pa, Vec3.subtract pt pr
                        let point port = position graph frame port |> Result.defaultWith invalidOp
                        let atRoot p = Vec3.distance p pa < 1e-6
                        let rootEdges =
                            program.Hinges |> List.filter (fun h -> atRoot (point h.A) && atRoot (point h.B))
                            |> List.map (fun h -> h.A.Body,h.B.Body)
                        let mutable reached = Set.singleton a.Body
                        for _ in 1..program.Bodies.Length do
                            for x,y in rootEdges do
                                if reached.Contains x || reached.Contains y then reached <- reached.Add(x).Add(y)
                        if not (atRoot pr) || not (reached.Contains root.Body) then
                            Error "The reference and driver roots must already share a physical hinge, possibly a shared pivot."
                        elif Vec3.length reference < 1e-6 || Vec3.length driver < 1e-6
                             || abs (Vec3.cross (Vec3.normalize reference) (Vec3.normalize driver)).Z < 1e-6 then
                            Error "A parallelogram must have nonzero sides and a nonsingular assembly pose."
                        elif graph.Bodies[root.Body].Grounded then Error "The existing driver arm must be moving."
                        else
                            let angle = frame.Poses[a.Body].Angle
                            let span = Rotation.apply (Rotation.aroundZ -angle) reference
                            let endPoint = Vec3.add pb driver
                            let parallelBody =
                                rigid parallelId pb.X pb.Y (atan2 driver.Y driver.X)
                                    [site "root" 0 0; site "tip" (Vec3.length driver) 0]
                            let output =
                                rigid outputId pt.X pt.Y angle
                                    ({Id=outputA; Position=Vec3.zero} :: {Id=outputB; Position=span} :: sites)
                            let candidate = {
                                program with
                                    Bodies=(program.Bodies |> List.map (fun body -> {body with Initial=frame.Poses[body.Id]})) @ [parallelBody;output]
                                    Hinges=program.Hinges @ [
                                        couple (prefix+"-base") b (port parallelId "root")
                                        couple (prefix+"-tip") (port parallelId "tip") (port outputId outputB)
                                        couple (prefix+"-driver") tip (port outputId outputA)
                                    ]
                            }
                            if Vec3.distance (transform output.Initial span) endPoint > 1e-6 then
                                Error "Parallelogram assembly did not preserve the reference orientation."
                            else
                                match compile candidate with
                                | Ok _ -> Ok candidate
                                | Error errors -> Error (String.concat "; " errors)
                    | Error error, _, _, _ | _, Error error, _, _ | _, _, Error error, _ | _, _, _, Error error -> Error error

    /// Add an orientation-preserving hinged output and transfer one existing logical foot to it.
    let addHingedFoot prefix reference driver outputPorts contact footId (program: MotionProgram) =
        if not (program.Feet |> List.exists (fun foot -> foot.Id = footId)) then Error $"Missing logical foot {footId}."
        else
            addParallelogram prefix reference driver outputPorts [contact] program
            |> Result.bind (fun candidate ->
                let feet =
                    candidate.Feet |> List.map (fun foot ->
                        if foot.Id = footId then {foot with Port=port (prefix+"-output") contact.Id} else foot)
                let candidate = {candidate with Feet=feet}
                match compile candidate with
                | Ok _ -> Ok candidate
                | Error errors -> Error (String.concat "; " errors))
