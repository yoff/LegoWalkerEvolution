namespace LegoWalkers

open System

/// Explicit parameter sharing over an expanded planar mechanism; never inferred from similar-looking legs.
module WalkerLegFamilies =
    let private reject message = invalidArg "leg-families" message
    let private number (x: float) =
        let rounded = Math.Round(x, 5)
        if rounded=0.0 then 0.0 else rounded
    let private xy (p: Vec3) = number p.X, number p.Y
    let private unique label values =
        if List.isEmpty values || List.length values <> Set.count (Set.ofList values) then
            reject $"{label} must be nonempty and unique."

    let canonical families =
        families |> List.map (fun family ->
            { family with
                Instances = family.Instances |> List.map (fun instance ->
                    { instance with
                        Bodies = List.sortBy _.Role instance.Bodies
                        Attachments = List.sortBy _.Role instance.Attachments }) |> List.sortBy _.Id })
        |> List.sortBy _.Id

    let members (program: MotionProgram) foot =
        let protectedIds =
            program.Bodies |> List.filter _.Grounded |> List.map _.Id
            |> Set.ofList |> Set.add program.Motor.Body
            |> Set.union (program.Transmissions |> List.collect (fun t -> [t.Input;t.Output]) |> Set.ofList)
        let rec visit reached = function
            | [] -> reached
            | id::rest when Set.contains id reached || Set.contains id protectedIds -> visit reached rest
            | id::rest ->
                let adjacent = program.Hinges |> List.choose (fun h ->
                    if h.A.Body=id then Some h.B.Body elif h.B.Body=id then Some h.A.Body else None)
                visit (Set.add id reached) (adjacent@rest)
        let output = program.Feet |> List.find (fun f -> f.Id=foot)
        visit Set.empty [output.Port.Body]

    let private boundaries (program: MotionProgram) bodies =
        program.Hinges
        |> List.filter (fun h -> Set.contains h.A.Body bodies || Set.contains h.B.Body bodies)
        |> List.collect (fun h -> [h.A;h.B])
        |> List.filter (fun p -> not (Set.contains p.Body bodies))
        |> List.distinct

    let instance id foot mirrored phase (program: MotionProgram) =
        let bodyIds = members program foot |> Set.toList
        let edges = boundaries program (Set.ofList bodyIds)
        let ground = program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
        let drives = edges |> List.filter (fun p -> not (ground.Contains p.Body))
        if drives.Length<>1 then reject "A reusable leg needs exactly one crank-tip input."
        let anchors = edges |> List.filter (fun p -> ground.Contains p.Body) |> List.sortBy (fun p ->
            let body = program.Bodies |> List.find (fun b -> b.Id=p.Body)
            let site = body.Sites |> List.find (fun s -> s.Id=p.Site)
            site.Position.X,site.Position.Y)
        { Id=id;Foot=foot;Mirrored=mirrored;Phase=phase
          Bodies=bodyIds |> List.mapi (fun n body -> {Role=$"link-{n}";Body=body})
          Attachments={Role="drive";Port=drives.Head}::(anchors |> List.mapi (fun n port -> {Role=$"anchor-{n}";Port=port})) }

    let rebind (program: MotionProgram) (original: LegInstance) =
        let current = instance original.Id original.Foot original.Mirrored original.Phase program
        let retained = original.Bodies |> List.filter (fun b -> current.Bodies |> List.exists (fun c -> c.Body=b.Body))
        let occupied = retained |> List.map _.Role |> Set.ofList
        let fresh = current.Bodies |> List.filter (fun b -> retained |> List.exists (fun c -> c.Body=b.Body) |> not)
        let names = Seq.initInfinite (fun n -> $"added-{n}") |> Seq.filter (occupied.Contains >> not)
        let preserved = current.Attachments |> List.choose (fun a ->
            original.Attachments |> List.tryFind (fun old -> old.Port=a.Port))
        let occupiedAttachments = preserved |> List.map _.Role |> Set.ofList
        let newAttachments = current.Attachments |> List.filter (fun a -> preserved |> List.exists (fun old -> old.Port=a.Port) |> not)
        let attachmentNames =
            Seq.initInfinite (fun n -> $"anchor-added-{n}") |> Seq.filter (occupiedAttachments.Contains >> not)
        let additions =
            List.zip (attachmentNames |> Seq.take newAttachments.Length |> Seq.toList) newAttachments
            |> List.map (fun (role,a) -> {a with Role=role})
        { current with
            Bodies=retained @ (List.zip (names |> Seq.take fresh.Length |> Seq.toList) fresh
                              |> List.map (fun (role,b) -> {b with Role=role}))
            Attachments=preserved @ additions }

    let private driveRatePhase (program: MotionProgram) (port: MotionPort) =
        let rec trace seen id =
            if Set.contains id seen then reject "Cyclic leg drive transmission."
            elif id=program.Motor.Body then 1.0,0.0
            else
                match program.Transmissions |> List.filter (fun t -> t.Output=id) with
                | [edge] ->
                    let rate,phase = trace (Set.add id seen) edge.Input
                    rate*edge.Ratio, phase*edge.Ratio+edge.Phase
                | _ -> reject $"Leg drive {id} needs an unambiguous motor transmission."
        let body = program.Bodies |> List.find (fun b -> b.Id=port.Body)
        let site = body.Sites |> List.find (fun s -> s.Id=port.Site)
        if Vec3.length site.Position<1e-6 then reject "A leg drive must identify its crank tip, not its axle center."
        let rate,phase = trace Set.empty port.Body
        rate,phase+atan2 site.Position.Y site.Position.X

    let motionSignature (program: MotionProgram) (instance: LegInstance) =
        let roles = instance.Bodies |> List.map (fun b -> b.Body,b.Role) |> Map.ofList
        let attachments = instance.Attachments |> List.map (fun a -> a.Port,a.Role) |> Map.ofList
        let port (p: MotionPort) =
            match Map.tryFind p.Body roles with
            | Some role -> role+"."+p.Site
            | None ->
                Map.tryFind p attachments |> Option.map ((+) "@")
                |> Option.defaultWith (fun () -> reject $"Missing attachment binding for {p.Body}/{p.Site}.")
        let drive = instance.Attachments |> List.find (fun a -> a.Role="drive") |> _.Port
        let input = program.Bodies |> List.find (fun b -> b.Id=drive.Body)
        let origin = input.Initial.Origin
        let radius = input.Sites |> List.find (fun site -> site.Id=drive.Site) |> _.Position |> Vec3.length
        let rate,_ = driveRatePhase program drive
        let bodies =
            instance.Bodies |> List.map (fun binding ->
                let body = program.Bodies |> List.find (fun b -> b.Id=binding.Body)
                binding.Role, (body.Sites |> List.map (fun site -> site.Id,xy site.Position) |> List.sort))
            |> List.sort
        let hinges =
            program.Hinges |> List.filter (fun h -> roles.ContainsKey h.A.Body || roles.ContainsKey h.B.Body)
            |> List.map (fun h -> [port h.A;port h.B] |> List.sort) |> List.sort
        let anchors =
            instance.Attachments |> List.choose (fun a ->
                let body = program.Bodies |> List.find (fun b -> b.Id=a.Port.Body)
                if not body.Grounded then None else
                    let site = body.Sites |> List.find (fun site -> site.Id=a.Port.Site)
                    Some (a.Role,xy (Vec3.subtract (Motion.transform body.Initial site.Position) origin)))
            |> List.sort
        let foot = program.Feet |> List.find (fun f -> f.Id=instance.Foot)
        Realization.json (bodies,hinges,anchors,port foot.Port,number foot.Radius,number radius,number rate)

    let shapeSignatureAt (poses: Map<string,MotionPose>) (genome: WalkerGenome) (instance: LegInstance) =
        instance.Bodies |> List.map (fun binding ->
            let pose = poses[binding.Body]
            let body = genome.Bodies |> List.find (fun b -> b.Body=binding.Body)
            let parts = body.Parts |> List.map (fun part ->
                let sockets =
                    (Catalog.get part.Part).Ports |> List.map (fun socket ->
                        let point = Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
                        let local = Rotation.apply (Rotation.aroundZ -pose.Angle) (Vec3.subtract point pose.Origin)
                        string socket.Kind,xy local) |> List.sort
                part.Part,sockets) |> List.sort
            binding.Role,parts)
        |> List.sort |> Realization.json

    let shapeSignature (program: MotionProgram) genome instance =
        shapeSignatureAt (program.Bodies |> List.map (fun b -> b.Id,b.Initial) |> Map.ofList) genome instance

    let validateAt (program: MotionProgram) (poses: Map<string,MotionPose>) (genome: WalkerGenome) =
        let families = defaultArg genome.LegFamilies []
        if not families.IsEmpty then unique "Leg family IDs" (families |> List.map _.Id)
        let instances = families |> List.collect _.Instances
        if not instances.IsEmpty then
            unique "Leg instance IDs" (instances |> List.map _.Id)
            unique "Family foot bindings" (instances |> List.map _.Foot)
            unique "Family body bindings" (instances |> List.collect _.Bodies |> List.map _.Body)
        for family in families do
            if String.IsNullOrWhiteSpace family.Id then reject "A leg family needs an ID."
            unique "Family instances" (family.Instances |> List.map _.Id)
            for instance in family.Instances do
                if String.IsNullOrWhiteSpace instance.Id || not (Double.IsFinite instance.Phase) then
                    reject "Leg instances require an ID and finite phase."
                unique "Leg body roles" (instance.Bodies |> List.map _.Role)
                unique "Leg attachment roles" (instance.Attachments |> List.map _.Role)
                unique "Leg attachment ports" (instance.Attachments |> List.map _.Port)
                if instance.Bodies |> List.exists (fun b ->
                    String.IsNullOrWhiteSpace b.Role
                    || not (program.Bodies |> List.exists (fun body -> body.Id=b.Body))
                    || not (genome.Bodies |> List.exists (fun body -> body.Body=b.Body)))
                   || instance.Attachments |> List.exists (fun a -> String.IsNullOrWhiteSpace a.Role)
                   || not (program.Feet |> List.exists (fun f -> f.Id=instance.Foot)) then
                    reject $"Instance {instance.Id} references a missing body or foot, or has an empty role."
                let expected = members program instance.Foot
                if expected <> (instance.Bodies |> List.map _.Body |> Set.ofList) then
                    reject $"Instance {instance.Id} must bind the complete independent passive leg."
                if (program.Feet |> List.filter (fun f -> expected.Contains f.Port.Body)).Length<>1 then
                    reject "A shared leg instance must have exactly one logical foot."
                if Set.ofList (boundaries program expected) <> (instance.Attachments |> List.map _.Port |> Set.ofList) then
                    reject $"Instance {instance.Id} must bind every external attachment exactly once."
                if instance.Attachments |> List.filter (fun a -> a.Role="drive") |> List.length <> 1 then
                    reject "Each leg instance requires one named drive attachment."
                for a in instance.Attachments do
                    match program.Bodies |> List.tryFind (fun b -> b.Id=a.Port.Body) with
                    | Some body when body.Sites |> List.exists (fun s -> s.Id=a.Port.Site) -> ()
                    | _ -> reject $"Missing attachment {a.Port.Body}/{a.Port.Site}."
            let reference = family.Instances.Head
            let signature = motionSignature program reference
            let shape = shapeSignatureAt poses genome reference
            let drive instance = instance.Attachments |> List.find (fun a -> a.Role="drive") |> _.Port
            let rate,phase = driveRatePhase program (drive reference)
            for instance in family.Instances.Tail do
                if motionSignature program instance<>signature || shapeSignatureAt poses genome instance<>shape then
                    reject $"Family {family.Id} no longer shares one leg design; split the family before an asymmetric edit."
                let otherRate,otherPhase = driveRatePhase program (drive instance)
                let delta = otherPhase-phase-(instance.Phase-reference.Phase)
                if abs(otherRate-rate)>1e-7 || abs(sin delta)>1e-7 || cos delta<0.9999999 then
                    reject $"Family {family.Id} changed its relative drive phases."

    let validate (program: MotionProgram) genome =
        validateAt program (program.Bodies |> List.map (fun b -> b.Id,b.Initial) |> Map.ofList) genome

    let bind id instances program genome =
        let candidate = { genome with LegFamilies = Some (canonical ({Id=id;Instances=instances}::defaultArg genome.LegFamilies [])) }
        let screened = Realization.screen program 72
                       |> Result.defaultWith (fun error -> reject (Realization.json error))
        validateAt program screened.Frames.Head.Poses candidate
        candidate

    let private adoptionInstance foot (program: MotionProgram) (genome: WalkerGenome) =
        let chassis = program.Bodies |> List.find _.Grounded
        let center = (genome.Bodies |> List.find (fun b -> b.Body=chassis.Id)).Parts |> List.averageBy _.Position.Z
        let choice = genome.Feet |> List.tryFind (fun f -> f.Foot=foot)
                     |> Option.defaultWith (fun () -> reject $"Unknown template foot {foot}.")
        let binding = instance foot foot (choice.Depth>center) 0.0 program
        let drive = binding.Attachments |> List.find (fun a -> a.Role="drive") |> _.Port
        let _,phase = driveRatePhase program drive
        {binding with Phase=phase}

    let adopt id feet (program: MotionProgram) (genome: WalkerGenome) =
        unique "Template feet" feet
        let instances = feet |> List.map (fun foot -> adoptionInstance foot program genome)
        bind id instances program genome

    type MigrationUnmatched = { Foot: string; Reason: string }
    type Migration = { Genome: WalkerGenome; Status: string; Families: LegFamily list; Unmatched: MigrationUnmatched list }

    let migrate (screened: ScreenedMotion) (genome: WalkerGenome) =
        match genome.LegFamilies with
        | Some families ->
            validateAt screened.Program screened.Frames.Head.Poses genome
            {Genome=genome;Status="existing-constraints-preserved";Families=families;Unmatched=[]}
        | None ->
            let program = screened.Program
            let eligible,unsupported =
                program.Feet |> List.sortBy _.Id |> List.map (fun foot ->
                    try
                        let binding = adoptionInstance foot.Id program genome
                        let bodies = binding.Bodies |> List.map _.Body |> Set.ofList
                        if (program.Feet |> List.filter (fun f -> bodies.Contains f.Port.Body)).Length<>1 then
                            reject "Passive branch contains more than one logical foot."
                        let key = motionSignature program binding,shapeSignatureAt screened.Frames.Head.Poses genome binding
                        Ok (key,binding)
                    with
                    | :? ArgumentException as error when error.ParamName="leg-families" ->
                        Error {Foot=foot.Id;Reason=error.Message})
                |> List.partition (function Ok _ -> true | _ -> false)
            let groups =
                eligible |> List.choose (function Ok value -> Some value | _ -> None)
                |> List.groupBy fst |> List.map (fun (_,items) -> items |> List.map snd)
            let shared,singletons = groups |> List.partition (fun instances -> instances.Length>1)
            let families =
                shared |> List.sortBy (fun instances -> instances.Head.Foot)
                |> List.mapi (fun n instances -> {Id=$"migrated-legs-{n+1}";Instances=instances})
                |> canonical
            let candidate = if families.IsEmpty then genome else {genome with LegFamilies=Some families}
            validateAt program screened.Frames.Head.Poses candidate
            { Genome=candidate
              Status=if families.IsEmpty then "no-matching-legs" else "converted"
              Families=families
              Unmatched=
                (unsupported |> List.choose (function Error reason -> Some reason | _ -> None))
                @ (singletons |> List.collect (List.map (fun instance ->
                    {Foot=instance.Foot;Reason="No exact matching motion, stock-part layout and drive rate."})))
                |> List.sortBy _.Foot }

    let split familyId instanceId genome =
        let families = defaultArg genome.LegFamilies []
        let family = families |> List.tryFind (fun f -> f.Id=familyId)
                     |> Option.defaultWith (fun () -> reject $"Unknown leg family {familyId}.")
        let instance = family.Instances |> List.tryFind (fun i -> i.Id=instanceId)
                       |> Option.defaultWith (fun () -> reject $"Unknown leg instance {instanceId}.")
        if family.Instances.Length<2 then reject "A singleton family cannot be split."
        let occupied = families |> List.map _.Id |> Set.ofList
        let id = Seq.initInfinite (fun n -> $"{familyId}-split-{n}") |> Seq.find (occupied.Contains >> not)
        let remaining =
            families |> List.map (fun f ->
                if f.Id=familyId then {f with Instances=f.Instances |> List.filter (fun i -> i.Id<>instanceId)} else f)
        { genome with LegFamilies = Some (canonical (remaining @ [{Id=id;Instances=[instance]}])) }
