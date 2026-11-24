namespace LegoWalkers.Spatial

open System
open LegoWalkers

type SpatialPose = { Origin: Vec3; Rotation: Rotation }
type SpatialBody = { Instance: string; Body: string }
type SpatialTemplate = { Id: string; Program: MotionProgram }
type SpatialBevelInput = { Source: SpatialBody; Ratio: float; Phase: float }
type SpatialInstance = {
    Id: string
    Template: string
    Carrier: SpatialBody
    Mount: SpatialPose
    Input: SpatialBevelInput
}
type SpatialFoot = { Id: string; Instance: string; Foot: string }
type SpatialProgram = {
    Root: MotionProgram
    Placement: SpatialPose
    Templates: SpatialTemplate list
    Instances: SpatialInstance list
    Feet: SpatialFoot list
}
type SpatialFrame = {
    InputAngle: float
    LocalFrames: Map<string, MotionFrame>
    Poses: Map<SpatialBody, SpatialPose>
    Feet: Map<string, Vec3>
    MaxResidual: float
}
type CompiledSpatialMotion = private {
    Program: SpatialProgram
    Graphs: Map<string, CompiledMotion>
    Programs: Map<string, MotionProgram>
    Ordered: SpatialInstance list
}

/// Kinematic composition only. A bevel input is an obligation for the physical
/// builder, not evidence that a gear mesh or a second motor exists.
module SpatialMotion =
    let root = "root"
    let identity : SpatialPose = { Origin = Vec3.zero; Rotation = Rotation.identity }
    let body instance id : SpatialBody = { Instance = instance; Body = id }

    let compose (outer: SpatialPose) (inner: SpatialPose) : SpatialPose =
        { Origin = Vec3.add outer.Origin (Rotation.apply outer.Rotation inner.Origin)
          Rotation = {
              X = Rotation.apply outer.Rotation inner.Rotation.X
              Y = Rotation.apply outer.Rotation inner.Rotation.Y
              Z = Rotation.apply outer.Rotation inner.Rotation.Z
          } }

    let transform (pose: SpatialPose) point =
        Vec3.add pose.Origin (Rotation.apply pose.Rotation point)

    let private validPose (pose: SpatialPose) =
        Vec3.finite pose.Origin && Rotation.valid pose.Rotation

    let private localPose (pose: MotionPose) : SpatialPose =
        { Origin = pose.Origin; Rotation = Rotation.aroundZ pose.Angle }

    let private groundedPivot (program: MotionProgram) id =
        let sites = program.Bodies |> List.map (fun b -> b.Id, b) |> Map.ofList
        program.Hinges |> List.tryPick (fun h ->
            let moving, other =
                if h.A.Body = id then h.A, h.B else h.B, h.A
            if moving.Body <> id || not sites[other.Body].Grounded then None
            else
                let local = sites[id].Sites |> List.find (fun s -> s.Id = moving.Site)
                if Vec3.length local.Position > 1e-8 then None
                else
                    let anchor = sites[other.Body].Sites |> List.find (fun s -> s.Id = other.Site)
                    Some (Motion.transform sites[other.Body].Initial anchor.Position))

    let compile (program: SpatialProgram) =
        let errors = ResizeArray<string>()
        let unique values = List.length values = (Set.ofList values).Count
        let validId = String.IsNullOrWhiteSpace >> not
        let templateIds = program.Templates |> List.map _.Id
        let instanceIds = program.Instances |> List.map _.Id
        if not (unique templateIds) || List.exists (fun id -> id = root || not (validId id)) templateIds then
            errors.Add "Spatial template IDs must be unique, nonempty, and different from root."
        if not (unique instanceIds) || List.exists (fun id -> id = root || not (validId id)) instanceIds then
            errors.Add "Spatial instance IDs must be unique, nonempty, and different from root."
        if not (validPose program.Placement) then errors.Add "Invalid root placement."
        let templates = program.Templates |> List.map (fun t -> t.Id, t.Program) |> Map.ofList
        let graphs = ResizeArray<string * CompiledMotion>()
        for id, p in (root, program.Root) :: (program.Templates |> List.map (fun t -> t.Id, t.Program)) do
            match Motion.compile p with
            | Error diagnostics ->
                let message = String.concat "; " diagnostics
                errors.Add $"{id}: {message}"
            | Ok graph -> graphs.Add (id, graph)
        for template in program.Templates do
            match template.Program.Bodies |> List.filter _.Grounded with
            | [ground] when Vec3.length ground.Initial.Origin < 1e-8 && abs ground.Initial.Angle < 1e-8 -> ()
            | _ -> errors.Add $"{template.Id}: a child requires one ground body at the local identity pose."
        let programs =
            (root, program.Root) :: (
                program.Instances |> List.choose (fun i ->
                    Map.tryFind i.Template templates |> Option.map (fun p -> i.Id, p)))
            |> Map.ofList
        for instance in program.Instances do
            if not (templates.ContainsKey instance.Template) then
                errors.Add $"{instance.Id}: unknown template {instance.Template}."
            if not (validPose instance.Mount)
               || abs instance.Mount.Origin.X > 1e-8 || abs instance.Mount.Origin.Y > 1e-8
               || abs instance.Mount.Rotation.Z.Z > 1e-8 then
                errors.Add $"{instance.Id}: a coaxial bevel mount needs a proper perpendicular plane on the carrier axis."
            if not (Double.IsFinite instance.Input.Ratio) || instance.Input.Ratio >= 0.0
               || not (Double.IsFinite instance.Input.Phase) then
                errors.Add $"{instance.Id}: external bevel input requires a finite negative ratio and finite phase."
            if instance.Input.Source.Instance <> instance.Carrier.Instance
               || instance.Input.Source.Body = instance.Carrier.Body then
                errors.Add $"{instance.Id}: input shaft and carrier must be distinct bodies in the same parent instance."
            for reference in [instance.Carrier; instance.Input.Source] do
                match Map.tryFind reference.Instance programs with
                | Some p when p.Bodies |> List.exists (fun b -> b.Id = reference.Body && not b.Grounded) -> ()
                | _ -> errors.Add $"{instance.Id}: missing moving parent body {reference.Instance}/{reference.Body}."
        if program.Feet.IsEmpty || program.Feet.Length > Motion.maximumFeet
           || not (unique (program.Feet |> List.map _.Id))
           || program.Feet |> List.exists (fun f -> not (validId f.Id))
           || not (unique (program.Feet |> List.map (fun f -> f.Instance, f.Foot))) then
            errors.Add $"Spatial programs require one to {Motion.maximumFeet} unique foot outputs."
        for foot in program.Feet do
            match Map.tryFind foot.Instance programs with
            | Some p when p.Feet |> List.exists (fun f -> f.Id = foot.Foot) -> ()
            | _ -> errors.Add $"Unknown spatial foot {foot.Instance}/{foot.Foot}."
        if errors.Count > 0 then Error (List.ofSeq errors)
        else
            for template in program.Templates do
                if groundedPivot template.Program template.Program.Motor.Body <> Some Vec3.zero then
                    errors.Add $"{template.Id}: the local input must rotate at the child frame origin."
            for instance in program.Instances do
                let parent = programs[instance.Carrier.Instance]
                match groundedPivot parent instance.Carrier.Body, groundedPivot parent instance.Input.Source.Body with
                | Some a, Some b when Vec3.distance a b < 1e-8 -> ()
                | _ -> errors.Add $"{instance.Id}: input shaft and carrier need coaxial ground bearings at their body origins."
            let ordered = ResizeArray<SpatialInstance>()
            let mutable reached = Set.singleton root
            let mutable pending = program.Instances
            let mutable progress = true
            while not pending.IsEmpty && progress do
                let ready, rest = pending |> List.partition (fun i -> reached.Contains i.Carrier.Instance)
                for instance in ready do
                    ordered.Add instance
                    reached <- reached.Add instance.Id
                progress <- not ready.IsEmpty
                pending <- rest
            if not pending.IsEmpty then errors.Add "Spatial carrier dependencies must form a rooted acyclic hierarchy."
            if errors.Count > 0 then Error (List.ofSeq errors)
            else
                let byTemplate = Map.ofSeq graphs
                Ok { Program = program; Programs = programs; Ordered = List.ofSeq ordered
                     Graphs = programs |> Map.map (fun id _ ->
                         if id = root then byTemplate[root]
                         else
                             let instance = program.Instances |> List.find (fun i -> i.Id = id)
                             byTemplate[instance.Template]) }

    let solve graph angle (previous: SpatialFrame option) =
        let expected = graph.Programs |> Map.keys |> Set.ofSeq
        if previous |> Option.exists (fun f -> Set.ofSeq f.LocalFrames.Keys <> expected) then
            Error "Previous spatial frame does not match this hierarchy."
        else
            let mutable locals = Map.empty
            let mutable poses = Map.empty
            let mutable failure = None
            let add id placement input =
                let before = previous |> Option.map (fun f -> f.LocalFrames[id])
                match Motion.solve graph.Graphs[id] input before with
                | Error error -> failure <- Some $"{id}: {error}"
                | Ok frame ->
                    locals <- locals.Add(id, frame)
                    for KeyValue(name, pose) in frame.Poses do
                        poses <- poses.Add(body id name, compose placement (localPose pose))
            add root graph.Program.Placement angle
            for instance in graph.Ordered do
                if failure.IsNone then
                    let parent = locals[instance.Carrier.Instance]
                    let shaft = parent.Poses[instance.Input.Source.Body].Angle
                    let carrier = parent.Poses[instance.Carrier.Body].Angle
                    let input =
                        instance.Input.Ratio*(shaft-carrier) + instance.Input.Phase
                        - graph.Programs[instance.Id].Motor.Phase
                    add instance.Id (compose poses[instance.Carrier] instance.Mount) input
            match failure with
            | Some error -> Error error
            | None ->
                let feet =
                    graph.Program.Feet |> List.map (fun foot ->
                        let p = graph.Programs[foot.Instance]
                        let output = p.Feet |> List.find (fun f -> f.Id = foot.Foot)
                        let owner = p.Bodies |> List.find (fun b -> b.Id = output.Port.Body)
                        let site = owner.Sites |> List.find (fun s -> s.Id = output.Port.Site)
                        foot.Id, transform poses[body foot.Instance owner.Id] site.Position)
                    |> Map.ofList
                Ok { InputAngle = angle; LocalFrames = locals; Poses = poses; Feet = feet
                     MaxResidual = locals.Values |> Seq.map _.MaxResidual |> Seq.max }

    let sample graph angles =
        let frames = ResizeArray<SpatialFrame>()
        let mutable previous = None
        let mutable failure = None
        for angle in angles do
            if failure.IsNone then
                match solve graph angle previous with
                | Error error -> failure <- Some error
                | Ok frame -> frames.Add frame; previous <- Some frame
        match failure with
        | Some error -> Error error
        | None -> Ok (List.ofSeq frames)
