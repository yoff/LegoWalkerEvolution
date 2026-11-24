namespace LegoWalkers

open System

type MotionChange = { Path: string; Before: float list; After: float list }
type WalkerDyad = {
    Prefix: string
    A: MotionPort
    B: MotionPort
    PivotA: string
    PivotB: string
    DepthA: float
    DepthB: float
    Direction: float
    Foot: string
}
type WalkerMutation = {
    Program: MotionProgram
    Operation: string
    Changes: MotionChange list
    RotatedPart: (string * float) option
    Dyad: WalkerDyad option
    RemovedBodies: string list
    FootReplacement: (string * MotionPort) option
    Physical: WalkerGenome option
    Components: WalkerMutation list
    FamilyUpdate: LegFamily list option
}

module WalkerEvolution =
    let version = "native-motion-mutations-v23"
    let selectionPolicy = "uniform-applicable-families-v1"
    let private near a b = abs (a-b) < 1e-6
    let private rotate angle vector = Rotation.apply (Rotation.aroundZ angle) vector
    let private compose a b = { X = Rotation.apply a b.X; Y = Rotation.apply a b.Y; Z = Rotation.apply a b.Z }
    let private local pose world = rotate -pose.Angle (Vec3.subtract world pose.Origin)
    let private worldPort s port = Motion.position s.Graph s.Frames.Head port |> Result.defaultWith invalidOp
    let private sockets (part: WalkerPlacement) =
        (Catalog.get part.Part).Ports |> List.map (fun port ->
            Vec3.add part.Position (Rotation.apply part.Rotation port.Position), port)
    let private onShaft (pin: WalkerPlacement) point =
        let delta = Vec3.subtract point pin.Position
        let lo, hi = Catalog.shaftRange pin.Part
        let axial = Vec3.dot delta pin.Rotation.X
        Vec3.length (Vec3.cross delta pin.Rotation.X) < 1e-6 && axial >= lo-1e-6 && axial <= hi+1e-6
    let private socketParts (s: ScreenedMotion) (g: WalkerGenome) port =
        let point = worldPort s port
        let ids =
            s.Program.Hinges |> List.filter (fun h -> h.A = port || h.B = port) |> List.map _.Id |> Set.ofList
        let pivots = g.Connections |> List.filter (fun c -> c.Hinges |> List.exists ids.Contains)
        (g.Bodies |> List.find (fun b -> b.Body = port.Body)).Parts
        |> List.filter (fun p -> sockets p |> List.exists (fun (position, _) ->
            near position.X point.X && near position.Y point.Y
            && pivots |> List.exists (fun c -> onShaft c.Placement position)))

    let mutations (s: ScreenedMotion) (g: WalkerGenome) =
        let driven = s.Program.Transmissions |> List.collect (fun t -> [t.Input; t.Output]) |> Set.ofList |> Set.add s.Program.Motor.Body
        let ports = s.Program.Hinges |> List.collect (fun h -> [h.A; h.B]) |> List.distinct |> List.sort
        let changeSite (port: MotionPort) next operation rotated =
            let b = s.Program.Bodies |> List.find (fun b -> b.Id = port.Body)
            let current = b.Sites |> List.find (fun site -> site.Id = port.Site)
            let bodies =
                s.Program.Bodies |> List.map (fun body ->
                    if body.Id <> b.Id then body
                    else
                        let sites = body.Sites |> List.map (fun site ->
                            if site.Id = port.Site then { site with Position = next } else site)
                        { body with Sites = sites })
            { Program = { s.Program with Bodies = bodies }
              Operation = operation
              Changes = [{ Path = $"bodies/{b.Id}/sites/{port.Site}/position"
                           Before = [current.Position.X; current.Position.Y]; After = [next.X; next.Y] }]
              RotatedPart = rotated; Dyad = None; RemovedBodies = []; FootReplacement = None; Physical = None
              Components = []; FamilyUpdate = None }
        [
            for port in ports do
                let b = s.Program.Bodies |> List.find (fun b -> b.Id = port.Body)
                if not b.Grounded then
                    match socketParts s g port with
                    | [part] ->
                        let current = (b.Sites |> List.find (fun site -> site.Id = port.Site)).Position
                        if not (driven.Contains b.Id) then
                            let available = sockets part |> List.map (fun (p, _) -> local s.Frames.Head.Poses[b.Id] p)
                            for delta in [-20.0; 20.0] do
                                let next = { current with X = current.X+delta }
                                if available |> List.exists (fun p -> near p.X next.X && near p.Y next.Y) then
                                    yield changeSite port next "move-hinge-site" None
                        elif Vec3.length current > 1e-6 then
                            let root = s.Frames.Head.Poses[b.Id].Origin
                            let rootSocket = sockets part |> List.exists (fun (p, socket) ->
                                socket.Kind = AxleHole && near p.X root.X && near p.Y root.Y)
                            if rootSocket then
                                for direction in [-1.0; 1.0] do
                                    let snap n = if abs n < 1e-8 then 0.0 else n
                                    let next = Vec3.create (snap (-direction*current.Y)) (snap (direction*current.X)) 0
                                    yield changeSite port next "branch-phase" (Some (part.Id, direction*Math.PI/2.0))
                    | _ -> ()
            for delta in [-Math.PI/2.0; Math.PI/2.0] do
                let phase = (s.Program.Motor.Phase+delta+2.0*Math.PI) % (2.0*Math.PI)
                yield { Program = { s.Program with Motor = { s.Program.Motor with Phase = phase } }
                        Operation = "input-phase"; RotatedPart = None; Dyad = None; RemovedBodies = []; FootReplacement = None; Physical = None
                        Changes = [{ Path = "motor/phase"; Before = [s.Program.Motor.Phase]; After = [phase] }]
                        Components = []; FamilyUpdate = None }
        ]

    let topologyMutations (s: ScreenedMotion) (g: WalkerGenome) =
        let chassis = s.Program.Bodies |> List.find _.Grounded
        let depths = (g.Bodies |> List.find (fun b -> b.Body = chassis.Id)).Parts |> List.map _.Position.Z
        let center = (List.min depths+List.max depths)/2.0
        let occupied = s.Program.Bodies |> List.map _.Id |> Set.ofList
        let prefix = Seq.initInfinite (fun n -> $"dyad-{n}") |> Seq.find (fun p -> not (occupied.Contains (p+"-a") || occupied.Contains (p+"-b")))
        let ports =
            [for c in g.Connections do
                if c.Hinges.Length = 1 then
                    let h = s.Program.Hinges |> List.find (fun h -> h.Id = c.Hinges.Head)
                    for port in [h.A; h.B] do
                        match socketParts s g port with
                        | [p] -> yield port, c, p
                        | _ -> ()]
            |> List.sortBy (fun (p,c,_) -> p.Body,p.Site,c.Group)
        let additions = [
            if s.Program.Bodies.Length <= 30 then
                for foot in s.Program.Feet do
                    for a, ca, pa in ports |> List.filter (fun (p,_,_) -> p.Body = foot.Port.Body) do
                        for b, cb, pb in ports do
                            if a.Body <> b.Body && ca.Group <> cb.Group
                               && abs (pa.Position.Z-pb.Position.Z) <= 20.000001
                               && (s.Program.Bodies |> List.find (fun body -> body.Id = b.Body) |> _.Grounded |> not) then
                                let distance = Vec3.distance (worldPort s a) (worldPort s b)
                                if distance > 20.0 && distance < 160.0 then
                                    for branch in [-1; 1] do
                                        match Motion.addDyad prefix a b 100.0 100.0 branch s.Program with
                                        | Error _ -> ()
                                        | Ok program ->
                                            let sign = if (pa.Position.Z+pb.Position.Z)/2.0 < center then -1.0 else 1.0
                                            let edge = if sign < 0.0 then min pa.Position.Z pb.Position.Z else max pa.Position.Z pb.Position.Z
                                            let output = Motion.port (prefix+"-a") "tip"
                                            yield {
                                                Program = { program with Feet = program.Feet |> List.map (fun f -> if f.Id = foot.Id then { f with Port = output } else f) }
                                                Operation = "add-dyad"; RotatedPart = None; RemovedBodies = []
                                                Components = []; FamilyUpdate = None
                                                FootReplacement = Some (foot.Id, output); Physical = None
                                                Dyad = Some { Prefix = prefix; A = a; B = b; PivotA = ca.Group; PivotB = cb.Group
                                                              DepthA = edge+sign*10.0; DepthB = edge+sign*20.0; Direction = sign; Foot = foot.Id }
                                                Changes = [
                                                    { Path = $"bodies/{prefix}-a,{prefix}-b"; Before = []; After = [100.0; 100.0] }
                                                    { Path = $"hinges/{prefix}"; Before = []; After = [3.0] }
                                                    { Path = $"feet/{foot.Id}/port:{foot.Port.Body}.{foot.Port.Site}->{output.Body}.{output.Site}"
                                                      Before = []; After = [1.0] }
                                                ] }
        ]
        let additions = additions |> List.truncate 24
        let removals = [
            let protectedBodies = s.Program.Motor.Body :: (s.Program.Transmissions |> List.collect (fun t -> [t.Input; t.Output])) |> Set.ofList
            for middle in s.Program.Hinges do
                let ids = [middle.A.Body; middle.B.Body]
                let attached id = s.Program.Hinges |> List.filter (fun h -> h.A.Body = id || h.B.Body = id)
                let removable id =
                    not (protectedBodies.Contains id)
                    && not (s.Program.Bodies |> List.find (fun b -> b.Id = id)).Grounded
                    && (attached id).Length = 2
                    && (g.Bodies |> List.find (fun b -> b.Body = id)).Parts.Length = 1
                if ids |> List.forall removable then
                    let external = ids |> List.map (fun id ->
                        let h = attached id |> List.find (fun h -> h.Id <> middle.Id)
                        h, (if ids |> List.contains h.A.Body then h.B else h.A))
                    let feet = s.Program.Feet |> List.filter (fun f -> ids |> List.contains f.Port.Body)
                    if feet.Length = 1 && (external |> List.map (snd >> _.Body) |> List.distinct |> List.length) = 2 then
                        let foot = feet.Head
                        let _, parentPort = external[if foot.Port.Body = ids.Head then 0 else 1]
                        let parentBody = s.Program.Bodies |> List.find (fun b -> b.Id = parentPort.Body)
                        let used = attached parentBody.Id |> List.collect (fun h -> [h.A;h.B]) |> Set.ofList
                        for site in parentBody.Sites do
                            let output = Motion.port parentBody.Id site.Id
                            if not (used.Contains output) then
                                let point = worldPort s output
                                let implemented = (g.Bodies |> List.find (fun b -> b.Body = parentBody.Id)).Parts
                                                  |> List.exists (fun p -> sockets p |> List.exists (fun (p,_) -> near p.X point.X && near p.Y point.Y))
                                if implemented then
                                    let removedHinges = middle.Id :: (external |> List.map (fst >> _.Id)) |> Set.ofList
                                    yield {
                                        Program = { s.Program with
                                                        Bodies = s.Program.Bodies |> List.filter (fun b -> not (List.contains b.Id ids))
                                                        Hinges = s.Program.Hinges |> List.filter (fun h -> not (removedHinges.Contains h.Id))
                                                        Feet = s.Program.Feet |> List.map (fun f -> if f.Id = foot.Id then { f with Port = output } else f) }
                                        Operation = "remove-dyad"; Dyad = None; RotatedPart = None; RemovedBodies = ids
                                        Components = []; FamilyUpdate = None
                                        FootReplacement = Some (foot.Id, output); Physical = None
                                        Changes = [{ Path = "bodies/"+String.concat "," ids; Before = [2.0]; After = [] }
                                                   { Path = $"feet/{foot.Id}/port:{foot.Port.Body}.{foot.Port.Site}->{output.Body}.{output.Site}"; Before = []; After = [1.0] }]
                                    }
        ]
        additions @ removals

    let physicalMutations (s: ScreenedMotion) (g: WalkerGenome) =
        let direction = {
            Program = s.Program; Operation = "motor-direction"
            Changes = [{Path="genome/speed";Before=[g.Speed];After=[-g.Speed]}]
            RotatedPart = None; Dyad = None; RemovedBodies = []; FootReplacement = None
            Physical = Some {g with Speed = -g.Speed}
            Components = []; FamilyUpdate = None
        }
        direction::((WalkerStructural.edits s g @ WalkerLayout.edits s g @ WalkerLoops.edits s g @ WalkerLevelFeet.edits s g) |> List.map (fun edit -> {
            Program = edit.Program; Operation = edit.Operation
            Changes = edit.Changes |> List.map (fun (path,before,after) -> { Path = path; Before = before; After = after })
            RotatedPart = None; Dyad = None; RemovedBodies = []; FootReplacement = None; Physical = Some edit.Genome
            Components = []; FamilyUpdate = None
        }))

    let private mergeList key equal baseline variants =
        let original = baseline |> List.map (fun x -> key x,x) |> Map.ofList
        let mutable edits = Map.empty
        let mutable compatible = true
        for variant in variants do
            let current = variant |> List.map (fun x -> key x,x) |> Map.ofList
            for id in Set.union (original |> Map.keys |> Set.ofSeq) (current |> Map.keys |> Set.ofSeq) do
                let before,after = Map.tryFind id original,Map.tryFind id current
                let same a b =
                    match a,b with
                    | None,None -> true
                    | Some x,Some y -> equal x y
                    | _ -> false
                if not (same before after) then
                    match Map.tryFind id edits with
                    | Some previous when not (same previous after) -> compatible <- false
                    | _ -> edits <- Map.add id after edits
        if not compatible then None else
            let result = edits |> Map.fold (fun state id value ->
                match value with Some x -> Map.add id x state | None -> Map.remove id state) original
            Some (result |> Map.values |> Seq.toList)

    let private mergePrograms (baseline: MotionProgram) (variants: MotionProgram list) =
        let bodies =
            let ids = (baseline.Bodies @ (variants |> List.collect _.Bodies)) |> List.map _.Id |> List.distinct
            ids |> List.map (fun bodyId ->
                let before = baseline.Bodies |> List.tryFind (fun b -> b.Id=bodyId)
                let after = variants |> List.map (fun p -> p.Bodies |> List.tryFind (fun b -> b.Id=bodyId))
                match before with
                | Some body when after |> List.forall Option.isSome ->
                    let changed = after |> List.choose id
                    let poses = changed |> List.map _.Initial |> List.filter ((<>) body.Initial) |> List.distinct
                    let sites = mergeList (fun (s: MotionSite) -> s.Id) (=) body.Sites (changed |> List.map _.Sites)
                    match poses,sites with
                    | [],Some sites -> Some (Some {body with Sites=sites})
                    | [pose],Some sites -> Some (Some {body with Initial=pose;Sites=sites})
                    | _ -> None
                | _ ->
                    mergeList (fun (b: MotionBody) -> b.Id) (=) (Option.toList before) (after |> List.map Option.toList)
                    |> Option.bind (function [] -> Some None | [body] -> Some (Some body) | _ -> None)
                    )
        if bodies |> List.exists Option.isNone then None else
        let motor = variants |> List.map _.Motor |> List.filter ((<>) baseline.Motor) |> List.distinct
        match mergeList (fun (h: MotionHinge) -> h.Id) (=) baseline.Hinges (variants |> List.map _.Hinges),
              mergeList _.Output (=) baseline.Transmissions (variants |> List.map _.Transmissions),
              mergeList (fun (f: MotionFoot) -> f.Id) (=) baseline.Feet (variants |> List.map _.Feet),motor with
        | Some hinges,Some transmissions,Some feet,([] | [_]) ->
            Some {Bodies=bodies |> List.choose id |> List.choose id;Hinges=hinges;Feet=feet;Transmissions=transmissions
                  Motor=List.tryHead motor |> Option.defaultValue baseline.Motor}
        | _ -> None

    let private familyMutations (s: ScreenedMotion) (g: WalkerGenome) (raw: WalkerMutation list) =
        let families = defaultArg g.LegFamilies []
        if families.IsEmpty then raw else
        let independent = {g with LegFamilies=None}
        let usable (program: MotionProgram) (instance: LegInstance) =
            match program.Feet |> List.tryFind (fun f -> f.Id=instance.Foot) with
            | None -> true
            | Some _ ->
                let members = WalkerLegFamilies.members program instance.Foot
                (program.Feet |> List.filter (fun f -> members.Contains f.Port.Body)).Length=1
        let rebound (program: MotionProgram) =
            families |> List.map (fun family ->
                {family with
                    Instances=family.Instances |> List.choose (fun instance ->
                        if program.Feet |> List.exists (fun f -> f.Id=instance.Foot) then
                            Some (WalkerLegFamilies.rebind program instance)
                        else None)}) |> List.filter (fun f -> not f.Instances.IsEmpty)
        let inheritClone (mutation: WalkerMutation) updated =
            match mutation.Changes |> List.tryFind (fun c -> c.Path.StartsWith("branches/") && c.Path.Contains("->")) with
            | Some change when mutation.Operation="duplicate-branch" ->
                let names = change.Path.Substring("branches/".Length).Split("->",StringSplitOptions.None)
                let oldFoot,newFoot = names[0],names[1]
                updated |> List.map (fun family ->
                    match family.Instances |> List.tryFind (fun i -> i.Foot=oldFoot) with
                    | None -> family
                    | Some original ->
                        let prefix = newFoot.Substring(0,newFoot.Length-oldFoot.Length)
                        let clone = WalkerLegFamilies.instance newFoot newFoot original.Mirrored original.Phase mutation.Program
                        let attachments = clone.Attachments |> List.map (fun a ->
                            let role = original.Attachments |> List.find (fun old -> a.Port.Site=prefix+old.Port.Site) |> _.Role
                            {a with Role=role})
                        let clone =
                            {clone with Bodies=original.Bodies |> List.map (fun b -> {b with Body=prefix+b.Body})
                                        Attachments=attachments}
                        {family with Instances=family.Instances @ [clone]})
            | _ -> updated
        let signature (program: MotionProgram) (physical: WalkerGenome option) (instance: LegInstance) =
            let poses =
                program.Bodies |> List.map (fun (b: MotionBody) ->
                    b.Id,(Map.tryFind b.Id s.Frames.Head.Poses |> Option.defaultValue b.Initial)) |> Map.ofList
            WalkerLegFamilies.motionSignature program instance,
            (physical |> Option.map (fun genome -> WalkerLegFamilies.shapeSignatureAt poses genome instance))
        let freshen (mutation: WalkerMutation) =
            match mutation.Dyad with
            | None -> mutation
            | Some d ->
                let name (id: string) = if id.StartsWith(d.Prefix+"-") then d.Foot+"--"+id else id
                let port (p: MotionPort) = {p with Body=name p.Body}
                let program = {mutation.Program with
                                Bodies=mutation.Program.Bodies |> List.map (fun b -> {b with Id=name b.Id})
                                Hinges=mutation.Program.Hinges |> List.map (fun h -> {h with Id=name h.Id;A=port h.A;B=port h.B})
                                Feet=mutation.Program.Feet |> List.map (fun f -> {f with Port=port f.Port})}
                {mutation with Program=program;Dyad=Some {d with Prefix=d.Foot+"--"+d.Prefix}
                               FootReplacement=mutation.FootReplacement |> Option.map (fun (id,p) -> id,port p)}
        let candidates =
            raw |> List.map freshen |> List.choose (fun mutation ->
                let changesGait =
                    mutation.Operation="branch-phase"
                    && (families |> List.filter (fun f -> f.Instances.Length>1) |> List.collect _.Instances
                        |> List.collect _.Attachments |> List.exists (fun a ->
                            mutation.Changes |> List.exists (fun change ->
                                change.Path = $"bodies/{a.Port.Body}/sites/{a.Port.Site}/position")))
                if changesGait then None else
                if families |> List.collect _.Instances |> List.forall (usable mutation.Program) |> not then None else
                let updated = rebound mutation.Program |> inheritClone mutation
                let differences = [
                    for family in updated do
                        let before = families |> List.find (fun f -> f.Id=family.Id)
                        for instance in family.Instances do
                            match before.Instances |> List.tryFind (fun i -> i.Id=instance.Id) with
                            | Some original when signature mutation.Program mutation.Physical instance
                                                 <> signature s.Program (mutation.Physical |> Option.map (fun _ -> independent)) original ->
                                yield family.Id,instance.Id,signature mutation.Program mutation.Physical instance
                            | _ -> ()
                ]
                Some (mutation,updated,differences))
        // Classify by the resulting template, not by world-space coordinates or instance names.
        let alreadyShared (mutation,updated,_) =
            updated |> List.forall (fun family ->
                family.Instances |> List.map (signature mutation.Program mutation.Physical) |> List.distinct |> List.length = 1)
        let safe = candidates |> List.filter alreadyShared |> List.map (fun (mutation,updated,_) ->
            {mutation with FamilyUpdate=Some updated
                           Physical=mutation.Physical |> Option.map (fun physical -> {physical with LegFamilies=Some updated})})
        let changes =
            candidates |> List.choose (fun (mutation,_,differences) ->
                match differences with
                | [family,instance,target] -> Some ((family,mutation.Operation,target),(instance,mutation))
                | _ -> None)
            |> List.groupBy fst
            |> List.choose (fun ((familyId,operation,_),entries) ->
                let family = families |> List.find (fun f -> f.Id=familyId)
                if family.Instances.Length<2 then None else
                let choices = entries |> List.map snd
                let components = family.Instances |> List.map (fun instance ->
                    choices |> List.tryFind (fun (id,_) -> id=instance.Id) |> Option.map snd)
                if components |> List.exists Option.isNone then None else
                let components = List.choose id components
                match mergePrograms s.Program (components |> List.map _.Program) with
                | None -> None
                | Some program ->
                    let updated = rebound program
                    Some {Program=program;Operation="shared-leg-"+operation
                          Changes=components |> List.collect _.Changes
                          RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Physical=None
                          Components=components;FamilyUpdate=Some updated})
        let splits = [
            for family in families do
                if family.Instances.Length>1 then
                    for instance in family.Instances do
                        yield {Program=s.Program;Operation="split-leg-family"
                               Changes=[{Path=$"legFamilies/{family.Id}/split/{instance.Id}";Before=[float family.Instances.Length];After=[1.0]}]
                               Physical=Some (WalkerLegFamilies.split family.Id instance.Id g)
                               RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Components=[];FamilyUpdate=None}
        ]
        let gait = [
            for mutation in raw do
                match mutation.Operation,mutation.RotatedPart with
                | "branch-phase",Some (part,angle) when angle>0.0 ->
                    for family in families do
                        for instance in family.Instances do
                            let drive = instance.Attachments |> List.find (fun a -> a.Role="drive") |> _.Port
                            if mutation.Changes |> List.exists (fun c -> c.Path=$"bodies/{drive.Body}/sites/{drive.Site}/position") then
                                let body = s.Program.Bodies |> List.find (fun b -> b.Id=drive.Body)
                                let site = body.Sites |> List.find (fun p -> p.Id=drive.Site)
                                let phase = (instance.Phase+Math.PI) % (2.0*Math.PI)
                                let program =
                                    {s.Program with
                                        Bodies=s.Program.Bodies |> List.map (fun b ->
                                            if b.Id<>drive.Body then b else
                                                {b with
                                                    Sites=b.Sites |> List.map (fun p ->
                                                        if p.Id=drive.Site then {p with Position=Vec3.scale -1.0 p.Position} else p)})}
                                let updated = families |> List.map (fun f ->
                                    {f with
                                        Instances=f.Instances |> List.map (fun i ->
                                            if i.Id=instance.Id then {i with Phase=phase} else i)})
                                yield {mutation with Program=program;Operation="gait-half-cycle";RotatedPart=Some(part,Math.PI)
                                                     FamilyUpdate=Some updated
                                                     Changes=[{Path=$"legFamilies/{family.Id}/instances/{instance.Id}/phase"
                                                               Before=[instance.Phase];After=[phase]}
                                                              {Path=$"bodies/{drive.Body}/sites/{drive.Site}/position"
                                                               Before=[site.Position.X;site.Position.Y];After=[-site.Position.X;-site.Position.Y]}]}
                | _ -> ()
        ]
        safe @ changes @ splits @ gait

    let lengthMutations s g =
        WalkerLengths.proposals s g |> List.map (fun (program,changes) ->
            {Program=program;Operation="limb-length"
             Changes=changes |> List.map (fun (path,before,after) -> {Path=path;Before=before;After=after})
             RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Physical=None;Components=[];FamilyUpdate=None})

    let private applicable s g =
        let independent = {g with LegFamilies=None}
        mutations s independent @ lengthMutations s independent @ topologyMutations s independent @ physicalMutations s independent
        |> familyMutations s g

    let chooseWithSelection s g =
        let families =
            applicable s g
            |> List.groupBy _.Operation
            |> List.sortBy fst
        if families.IsEmpty then invalidArg "parent" "No supported native motion mutations exist."
        fun seed attempt ->
            if seed < 0 || attempt < 0 || attempt > 63 then invalidArg "proposal" "Seed must be nonnegative; attempt must be 0..63."
            let random = Random seed
            let rec draw remaining =
                let _, choices = families[random.Next families.Length]
                let mutation = choices[random.Next choices.Length]
                if remaining = 0 then mutation else draw (remaining-1)
            let mutation = draw attempt
            mutation, {| policy = selectionPolicy; selectedFamily = mutation.Operation
                         availableFamilies = families |> List.map (fun (operation, choices) -> {| operation = operation; edits = choices.Length |}) |}

    let choose s g seed attempt = chooseWithSelection s g seed attempt |> fst

    let editsWithPolicy policy (s: ScreenedMotion) (g: WalkerGenome) =
        let templates = [
            let bound = defaultArg g.LegFamilies [] |> List.collect _.Instances |> List.map _.Foot |> Set.ofList
            let occupied = defaultArg g.LegFamilies [] |> List.map _.Id |> Set.ofList
            let ground = s.Program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
            for foot in s.Program.Feet do
                if not (bound.Contains foot.Id) then
                    let members = WalkerLegFamilies.members s.Program foot.Id
                    let boundary =
                        s.Program.Hinges |> List.filter (fun h -> members.Contains h.A.Body || members.Contains h.B.Body)
                        |> List.collect (fun h -> [h.A;h.B])
                        |> List.filter (fun p -> not (members.Contains p.Body) && not (ground.Contains p.Body))
                        |> List.distinct
                    if not members.IsEmpty && boundary.Length=1
                       && (s.Program.Feet |> List.filter (fun f -> members.Contains f.Port.Body)).Length=1 then
                        let name = Seq.initInfinite (fun n -> $"template-{foot.Id}-{n}") |> Seq.find (occupied.Contains >> not)
                        let adopted = WalkerLegFamilies.adopt name [foot.Id] s.Program g
                        yield {Program=s.Program;Physical=Some adopted;Operation="share-leg-template"
                               Changes=[{Path=$"feet/{foot.Id}/template";Before=[];After=[1.0]}]
                               RotatedPart=None;Dyad=None;RemovedBodies=[];FootReplacement=None;Components=[];FamilyUpdate=None}
        ]
        let construction =
            WalkerConstruction.choicesWithPolicy policy s g
            |> Seq.map (fun choice ->
                { Program = s.Program; Operation = choice.Motif
                  Changes = [{ Path = "connections/"+choice.Pivot; Before = []; After = [] }]
                  RotatedPart = None; Dyad = None; RemovedBodies = []; FootReplacement = None
                  Physical = Some choice.Genome; Components = []; FamilyUpdate = None })
            |> Seq.toList
        applicable s g @ construction @ templates
        |> List.map (fun mutation ->
            let level =
                if mutation.Program = s.Program then "realization"
                elif mutation.Physical.IsSome then "coupled"
                else "motion"
            Export.sha256 (Realization.json mutation), level, mutation)
        |> List.distinctBy (fun (id,_,_) -> id)
        |> List.sortBy (fun (id,level,mutation) ->
            let priority (suffix: string) =
                mutation.Changes |> List.tryPick (fun change ->
                    if change.Path.EndsWith(suffix) then List.tryHead change.After else None)
                |> Option.defaultValue 0.0
            level,mutation.Operation,-(priority "/l-layout-preference"),priority "/level-foot-rank",id)

    let private realizeSingleWithPolicy policy (parent: ScreenedMotion) (genome: WalkerGenome) (mutation: WalkerMutation) (child: ScreenedMotion) limit =
        if limit < 1 || limit > 4 then invalidArg "limit" "A native proposal permits one to four physical realizations."
        match mutation.Physical with
        | Some physical ->
            Ok (WalkerConstruction.alternativesWithPolicy policy child { physical with MechanismId = child.MechanismId } limit
                |> List.map _.Genome)
        | None ->
          try
            let transform id (p: WalkerPlacement) =
                let before, after = parent.Frames.Head.Poses[id], child.Frames.Head.Poses[id]
                let source =
                    match mutation.RotatedPart with
                    | Some (part, angle) when part = p.Id ->
                        { p with Position = Vec3.add before.Origin (rotate angle (Vec3.subtract p.Position before.Origin))
                                 Rotation = compose (Rotation.aroundZ angle) p.Rotation }
                    | _ -> p
                { source with Position = Motion.transform after (local before source.Position)
                              Rotation = compose (Rotation.aroundZ (after.Angle-before.Angle)) source.Rotation }
            let hinges = child.Program.Hinges |> List.map (fun h -> h.Id, h) |> Map.ofList
            let bodies = genome.Bodies |> List.filter (fun b -> not (List.contains b.Body mutation.RemovedBodies))
                                      |> List.map (fun b -> { b with Parts = b.Parts |> List.map (transform b.Body) })
            let connections = genome.Connections
                              |> List.map (fun c -> { c with Hinges = c.Hinges |> List.filter hinges.ContainsKey })
                              |> List.filter (fun c -> not c.Hinges.IsEmpty)
                              |> List.map (fun c ->
                let h = hinges[c.Hinges.Head]
                let point = worldPort child h.A
                let rotation =
                    if (Catalog.get c.Placement.Part).Connector = Some Axle then
                        let id = if (child.Program.Bodies |> List.find (fun b -> b.Id = h.A.Body)).Grounded then h.B.Body else h.A.Body
                        let delta = child.Frames.Head.Poses[id].Angle-parent.Frames.Head.Poses[id].Angle
                        compose (Rotation.aroundZ delta) c.Placement.Rotation
                    else c.Placement.Rotation
                { c with Placement = { c.Placement with Position = { point with Z = c.Placement.Position.Z }; Rotation = rotation } })
            let mounts = genome.Mounts |> List.map (fun pin ->
                let owners = genome.Bodies |> List.filter (fun b -> b.Parts |> List.exists (fun p ->
                    sockets p |> List.exists (fun (point, socket) -> socket.Kind = RoundHole && onShaft pin point)))
                match owners with
                | [owner] -> transform owner.Body pin
                | _ -> invalidArg "mount" $"Cannot uniquely associate rigid mount {pin.Id} with a parent body.")
            let mutable first = { genome with MechanismId = child.MechanismId; Bodies = bodies; Connections = connections; Mounts = mounts }
            if mutation.Operation="limb-length" then
                first <- WalkerLengths.rebuild policy parent child first
            match mutation.Dyad with
            | Some d ->
                let link name depth =
                    let pose = child.Frames.Head.Poses[name]
                    let normal = Vec3.create 0 0 -d.Direction
                    let rotation = Rotation.beamWithNormal normal (rotate pose.Angle (Vec3.create 1 0 0))
                    let part = {
                        Id = name+"-part"; Part = "32063.dat"; Color = 4
                        Position = Vec3.subtract { pose.Origin with Z = depth } (Rotation.apply rotation (Vec3.create 0 0 -50))
                        Rotation = rotation
                    }
                    { Body = name; Parts = [part] }
                let spanPivot (c: WalkerPivotChoice) newHinge depth =
                    let old = c.Placement
                    let lo,hi = Catalog.shaftRange old.Part
                    let ends = [old.Position.Z+lo*old.Rotation.X.Z; old.Position.Z+hi*old.Rotation.X.Z; depth-5.0;depth+5.0]
                    let low,high = List.min ends,List.max ends
                    if high-low > 60.000001 then invalidArg "dyad" "Dyad root exceeds the supported three-body pin span."
                    { c with Hinges = c.Hinges @ [newHinge]
                             Placement = { old with Part = "32556.dat"; Position = { old.Position with Z = (low+high)/2.0 } } }
                let tip = worldPort child (Motion.port (d.Prefix+"-a") "tip")
                let middle = {
                    Group = d.Prefix+"-middle"; Hinges = [d.Prefix+"-middle"]
                    Placement = { Id = "pivot-"+d.Prefix+"-middle"; Part = "32002.dat"; Color = 8
                                  Position = { tip with Z = (d.DepthA+d.DepthB)/2.0 }
                                  Rotation = { X = Vec3.create 0 0 -d.Direction
                                               Y = Vec3.create 0 1 0; Z = Vec3.create d.Direction 0 0 } } }
                first <- { first with Bodies = first.Bodies @ [link (d.Prefix+"-a") d.DepthA; link (d.Prefix+"-b") d.DepthB]
                                      Connections = (first.Connections |> List.map (fun c ->
                                          if c.Group = d.PivotA then spanPivot c (d.Prefix+"-root-a") d.DepthA
                                          elif c.Group = d.PivotB then spanPivot c (d.Prefix+"-root-b") d.DepthB
                                          else c)) @ [middle]
                                      Feet = first.Feet |> List.map (fun f -> if f.Foot = d.Foot then { f with Part = d.Prefix+"-a-part"; Depth = d.DepthA } else f) }
            | None ->
                match mutation.FootReplacement with
                | Some (id, port) ->
                    let point = worldPort child port
                    let part = (first.Bodies |> List.find (fun b -> b.Body = port.Body)).Parts
                               |> List.find (fun p -> sockets p |> List.exists (fun (p,_) -> near p.X point.X && near p.Y point.Y))
                    first <- { first with Feet = first.Feet |> List.map (fun f -> if f.Foot = id then { f with Part = part.Id; Depth = part.Position.Z } else f) }
                | None -> ()
            let validated =
                if mutation.Operation="limb-length" then WalkerRealization.compile child first |> Result.map _.Genome
                else Ok first
            validated |> Result.map (fun physical ->
                WalkerConstruction.alternativesWithPolicy policy child physical limit |> List.map _.Genome)
          with :? ArgumentException as error ->
            Error { Gate = "lego-realizability"; Outcome = "reject"; Code = "unsupported-mutation-realization"; Diagnostics = [error.Message] }

    let private realizeFamilyWithPolicy policy (parent: ScreenedMotion) (genome: WalkerGenome) (mutation: WalkerMutation) (child: ScreenedMotion) limit =
        if limit<1 || limit>4 then invalidArg "limit" "A native proposal permits one to four physical realizations."
        let independent = {genome with LegFamilies=None}
        let components = if mutation.Components.IsEmpty then [mutation] else mutation.Components
        let results = components |> List.map (fun edit ->
            let screened =
                if mutation.Components.IsEmpty then Ok child
                else Realization.screen edit.Program (parent.Frames.Length-1)
            screened |> Result.bind (fun frame ->
                realizeSingleWithPolicy policy parent independent edit frame 1
                |> Result.map List.exactlyOne))
        match results |> List.tryPick (function Error error -> Some error | _ -> None) with
        | Some error -> Error error
        | None ->
            let variants = results |> List.choose (function Ok value -> Some value | _ -> None)
            let nearVec a b = Vec3.distance a b<1e-5
            let nearPart (a: WalkerPlacement) (b: WalkerPlacement) =
                a.Id=b.Id && a.Part=b.Part && a.Color=b.Color
                && nearVec a.Position b.Position
                && nearVec a.Rotation.X b.Rotation.X && nearVec a.Rotation.Y b.Rotation.Y && nearVec a.Rotation.Z b.Rotation.Z
            let bodyIds = (independent.Bodies @ (variants |> List.collect _.Bodies)) |> List.map _.Body |> List.distinct
            let bodies = bodyIds |> List.map (fun bodyId ->
                let old = independent.Bodies |> List.tryFind (fun b -> b.Body=bodyId)
                let choices = variants |> List.map (fun g -> g.Bodies |> List.tryFind (fun b -> b.Body=bodyId))
                match old with
                | Some body when choices |> List.forall Option.isSome ->
                    mergeList (fun (p: WalkerPlacement) -> p.Id) nearPart body.Parts (choices |> List.choose id |> List.map _.Parts)
                    |> Option.map (fun parts -> Some {body with Parts=parts})
                | _ ->
                    mergeList (fun (b: WalkerBodyChoice) -> b.Body) (=) (Option.toList old) (choices |> List.map Option.toList)
                    |> Option.bind (function [] -> Some None | [body] -> Some (Some body) | _ -> None))
            let nearPivot (a: WalkerPivotChoice) (b: WalkerPivotChoice) =
                a.Group=b.Group && a.Hinges=b.Hinges && nearPart a.Placement b.Placement
            let pivots = mergeList _.Group nearPivot independent.Connections (variants |> List.map _.Connections)
            let mounts = mergeList (fun (p: WalkerPlacement) -> p.Id) nearPart independent.Mounts (variants |> List.map _.Mounts)
            let feet = mergeList (fun (f: WalkerFootChoice) -> f.Foot) (=) independent.Feet (variants |> List.map _.Feet)
            match bodies |> List.forall Option.isSome,pivots,mounts,feet with
            | true,Some pivots,Some mounts,Some feet ->
                let first = variants.Head
                let families =
                    if mutation.Operation="split-leg-family" then first.LegFamilies
                    else mutation.FamilyUpdate |> Option.orElse genome.LegFamilies
                let combined =
                    {first with MechanismId=child.MechanismId;LegFamilies=families |> Option.map WalkerLegFamilies.canonical
                                Bodies=bodies |> List.choose id |> List.choose id;Connections=pivots;Mounts=mounts;Feet=feet}
                WalkerRealization.compile child combined
                |> Result.map (fun _ ->
                    WalkerConstruction.alternativesWithPolicy policy child combined limit
                    |> List.map _.Genome)
            | _ ->
                let conflicts =
                    [ if bodies |> List.exists Option.isNone then "body parts"
                      if pivots.IsNone then "pivots"
                      if mounts.IsNone then "rigid mounts"
                      if feet.IsNone then "foot definitions" ] |> String.concat ", "
                Error {Gate="lego-realizability";Outcome="reject";Code="conflicting-shared-leg-realizations"
                       Diagnostics=[$"Shared leg instances require incompatible changes to common hardware: {conflicts}."]}

    let realizeWithPolicy policy parent genome mutation child limit =
        let result =
            if mutation.Operation="shared-leg-limb-length" then
                let combined = {mutation with Operation="limb-length";Components=[]}
                let physical = {genome with LegFamilies=mutation.FamilyUpdate |> Option.orElse genome.LegFamilies}
                realizeSingleWithPolicy policy parent physical combined child limit
            elif genome.LegFamilies.IsNone && mutation.Components.IsEmpty then
                realizeSingleWithPolicy policy parent genome mutation child limit
            else realizeFamilyWithPolicy policy parent genome mutation child limit
        if not(List.contains mutation.Operation ["replace-double-parallelogram-foot";"shared-leg-replace-double-parallelogram-foot"]) then result else
            result |> Result.bind (fun candidates ->
                candidates |> List.fold (fun accumulated candidate ->
                    accumulated |> Result.bind (fun previous ->
                        WalkerFootPlates.prepare child candidate |> Result.map (fun (prepared,_) -> previous @ [prepared]))) (Ok []))

    let realize parent genome mutation child limit =
        realizeWithPolicy WalkerBuilder.defaultPolicy parent genome mutation child limit
