namespace LegoWalkers

open System.IO
open System.Text.Json
open System.Text.Json.Serialization

type WalkerConstructionFeedback = {
    Kind: string
    Parts: string list
    Bodies: string list
}

type WalkerConstructionChoice = {
    Genome: WalkerGenome
    Motif: string
    Pivot: string
    Members: string list
    ClearanceImprovement: float
    AffectedBodies: string list
    LayerOffset: float
}

module WalkerConstruction =
    let version = "technic-construction-v11"

    let readFeedback path =
        let options = JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                                            UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
                                            RespectRequiredConstructorParameters = true)
        JsonSerializer.Deserialize<WalkerConstructionFeedback>(File.ReadAllText path, options)

    let internal members (program: MotionProgram) (pivot: WalkerPivotChoice) =
        program.Hinges
        |> List.filter (fun h -> List.contains h.Id pivot.Hinges)
        |> List.collect (fun h -> [h.A.Body; h.B.Body])
        |> List.distinct |> List.sort

    let private engaged (g: WalkerGenome) pin =
        g.Bodies
        |> List.filter (fun b -> b.Parts |> List.exists (WalkerStructural.engaged pin))
        |> List.map _.Body |> List.sort

    let private span part =
        let lo, hi = Catalog.shaftRange part
        hi-lo

    let private withinContactCapacity (g: WalkerGenome) pin =
        let contacts = g.Bodies |> List.collect _.Parts |> List.filter (WalkerStructural.engaged pin) |> List.length
        // Supported smooth pins have mesh-length capacities of two, two and three contacts.
        let capacity = max 2 (int (System.Math.Round(span pin.Part/20.0)))
        contacts <= capacity

    let private midpoint (pin: WalkerPlacement) =
        let lo, hi = Catalog.shaftRange pin.Part
        Vec3.add pin.Position (Vec3.scale ((lo+hi)/2.0) pin.Rotation.X)

    let private replace (g: WalkerGenome) (pivot: WalkerPivotChoice) =
        { g with Connections = g.Connections |> List.map (fun c -> if c.Group = pivot.Group then pivot else c) }

    let internal socketEnvelope part =
        match Realization.bounds part with
        | Some bounds -> Some bounds
        | None when part = "6632.dat" ->
            Some (Vec3.create -9 -5 -9, Vec3.create 9 5 29)
        | None -> None

    let internal axleWallPoints (pin: WalkerPlacement) (part: WalkerPlacement) =
        let half = Catalog.socketHalfThickness part.Part
        WalkerStructural.engagedPorts pin part |> List.collect (fun (_,point) ->
            [Vec3.add point (Vec3.scale -half pin.Rotation.X);Vec3.add point (Vec3.scale half pin.Rotation.X)])

    let internal socketWallCovered (pin: WalkerPlacement) (part: WalkerPlacement) =
        if (Catalog.get pin.Part).Connector = Some Axle then
            let lo,hi = Catalog.shaftRange pin.Part
            axleWallPoints pin part |> List.forall (fun point ->
                let along = Vec3.dot (Vec3.subtract point pin.Position) pin.Rotation.X
                along >= lo-1e-6 && along <= hi+1e-6)
        else
            match socketEnvelope part.Part with
            | None -> false
            | Some (low,high) ->
                let lo,hi = Catalog.shaftRange pin.Part
                [for x in [low.X;high.X] do
                    for y in [low.Y;high.Y] do
                        for z in [low.Z;high.Z] ->
                            Vec3.add part.Position (Rotation.apply part.Rotation (Vec3.create x y z))]
                |> List.forall (fun point ->
                    let along = Vec3.dot (Vec3.subtract point pin.Position) pin.Rotation.X
                    along >= lo-1e-6 && along <= hi+1e-6)

    let internal supportedStack (g: WalkerGenome) pin =
        withinContactCapacity g pin
        && g.Bodies |> List.collect _.Parts |> List.filter (WalkerStructural.engaged pin)
           |> List.forall (socketWallCovered pin)

    let internal pivotSupported program g pivot =
        engaged g pivot.Placement = members program pivot && supportedStack g pivot.Placement

    let internal beamFits (source: WalkerPlacement) requirements = [
        for part in Catalog.planarBeamParts do
          for position,rotation in Catalog.fitPlanarPart part (Vec3.create 0 0 1) (requirements |> List.map fst) do
            let candidate = {source with Part=part;Position=position;Rotation=rotation}
            if requirements |> List.forall (fun (point,kind) ->
                (Catalog.get part).Ports |> List.exists (fun socket ->
                    socket.Kind=kind
                    && Vec3.distance point (Vec3.add position (Rotation.apply rotation socket.Position))<1e-5)) then
                yield candidate
    ]

    let private beamGroup (g: WalkerGenome) body =
        defaultArg g.LegFamilies []
        |> List.tryPick (fun family ->
            family.Instances |> List.tryPick (fun instance ->
                instance.Bodies |> List.tryFind (fun binding -> binding.Body=body)
                |> Option.map (fun binding -> family.Id,binding.Role)))
        |> Option.defaultValue ("",body)

    let internal fitBeams (s: ScreenedMotion) (g: WalkerGenome) (choices: (string * WalkerPlacement list) list) =
        let floor = WalkerClearance.initialFootFloor g
        let signature body (part: WalkerPlacement) =
            let pose = s.Frames.Head.Poses[body]
            part.Part, ((Catalog.get part.Part).Ports |> List.map (fun socket ->
                let world = Vec3.add part.Position (Rotation.apply part.Rotation socket.Position)
                let local = Rotation.apply (Rotation.aroundZ -pose.Angle) (Vec3.subtract world pose.Origin)
                socket.Kind,System.Math.Round(local.X,6),System.Math.Round(local.Y,6)) |> List.sort)
        let inherited body (p: WalkerPlacement) =
            let source = (g.Bodies |> List.find (fun b -> b.Body=body)).Parts.Head
            (if p.Part=source.Part then 0 else 1),(Catalog.get p.Part).Ports.Length,
            Vec3.distance p.Rotation.Y source.Rotation.Y,Vec3.distance p.Position source.Position,p.Part
        let replacements =
            choices |> List.groupBy (fst >> beamGroup g) |> List.collect (fun (_,group) ->
                let keys =
                    group |> List.map (fun (body,parts) -> parts |> List.map (signature body) |> Set.ofList)
                    |> Set.intersectMany
                if keys.IsEmpty then invalidArg "limb" "No supported stock beam realizes the revised sockets consistently across the leg family."
                let candidates = [
                    for key in keys do
                        let selected =
                            group |> List.map (fun (body,parts) ->
                                body,(parts |> List.filter (fun p -> signature body p=key) |> List.minBy (inherited body)))
                        let intrusion = selected |> List.sumBy (snd >> WalkerClearance.initialGroundIntrusion floor)
                        let preferences = selected |> List.map (fun (body,p) -> inherited body p)
                        yield (intrusion,preferences,key),selected
                ]
                candidates |> List.minBy fst |> snd)
            |> Map.ofList
        {g with Bodies=g.Bodies |> List.map (fun body ->
            match Map.tryFind body.Body replacements with
            | Some part -> {body with Parts=[part]}
            | None -> body)}

    let private initialGroundRepairs (s: ScreenedMotion) (g: WalkerGenome) targeted =
        let groups = g.Bodies |> List.filter targeted |> List.map (fun b -> beamGroup g b.Body) |> Set.ofList
        let protectedIds = WalkerStructural.protectedBodies s.Program
        let footParts = g.Feet |> List.collect WalkerFootChoice.contacts |> List.map _.Part |> Set.ofList
        let choices = [
            for body in g.Bodies do
                if groups.Contains(beamGroup g body.Body) && not (protectedIds.Contains body.Body)
                   && body.Parts.Length=1 && List.contains body.Parts.Head.Part Catalog.planarBeamParts
                   && not (footParts.Contains body.Parts.Head.Id)
                   && not (g.Mounts |> List.exists (fun pin -> WalkerStructural.engaged pin body.Parts.Head)) then
                    let source = body.Parts.Head
                    let requirements =
                        (s.Program.Bodies |> List.find (fun b -> b.Id=body.Body)).Sites
                        |> List.map (fun site ->
                            let point = Motion.transform s.Frames.Head.Poses[body.Body] site.Position
                            let point = {point with Z=source.Position.Z}
                            (Catalog.get source.Part).Ports |> List.tryFind (fun socket ->
                                Vec3.distance point (Vec3.add source.Position (Rotation.apply source.Rotation socket.Position))<1e-5)
                            |> Option.map (fun socket -> point,socket.Kind))
                    if requirements |> List.forall Option.isSome then
                        let required = requirements |> List.choose id
                        if required |> List.map fst |> List.distinct |> List.length >= 2 then
                            yield body.Body,beamFits source required
        ]
        if choices.IsEmpty || choices |> List.exists (snd >> List.isEmpty) then [] else
        let candidate = fitBeams s g choices
        let affected = candidate.Bodies |> List.filter (fun b -> b<>(g.Bodies |> List.find (fun old -> old.Body=b.Body))) |> List.map _.Body
        let floor = WalkerClearance.initialFootFloor g
        let intrusion (genome: WalkerGenome) =
            genome.Bodies |> List.filter (fun b -> choices |> List.exists (fun (id,_) -> id=b.Body))
            |> List.collect _.Parts |> List.sumBy (WalkerClearance.initialGroundIntrusion floor)
        let improvement = intrusion g-intrusion candidate
        let covered =
            candidate.Connections |> List.forall (fun pivot ->
                (Catalog.get pivot.Placement.Part).Connector<>Some FrictionlessPin
                || not (members s.Program pivot |> List.exists (fun body -> List.contains body affected))
                || pivotSupported s.Program candidate pivot)
        if improvement<=1e-6 || not covered then [] else
        match WalkerRealization.compile s candidate with
        | Error _ -> []
        | Ok _ ->
            [{Genome=candidate;Motif="initial-foot-clearance-beam-fit";Pivot="";Members=[]
              ClearanceImprovement=improvement;AffectedBodies=affected;LayerOffset=0.0}]

    let private motif (before: WalkerPlacement) (after: WalkerPlacement) =
        if span after.Part < span before.Part then "smooth-pivot-shorter-pin"
        elif after.Part = before.Part then "smooth-pivot-reversed-pin"
        else "smooth-pivot-longer-pin"

    let internal clearanceRisk (g: WalkerGenome) expected (pin: WalkerPlacement) =
        let center = midpoint pin
        let half = span pin.Part/2.0
        // A half-stud collar envelope ranks candidates; it is never a collision gate.
        let radius axis = half*abs axis + 10.0*sqrt (max 0.0 (1.0-axis*axis))
        let extent = Vec3.create (radius pin.Rotation.X.X) (radius pin.Rotation.X.Y) (radius pin.Rotation.X.Z)
        let low,high = Vec3.subtract center extent,Vec3.add center extent
        let overlap a b c d = max 0.0 (min b d-max a c)
        g.Bodies |> List.sumBy (fun body ->
            body.Parts |> List.sumBy (fun part ->
                if List.contains body.Body expected && WalkerStructural.engaged pin part then 0.0
                else
                    match Realization.bounds part.Part with
                    | None -> 0.0
                    | Some (lo,hi) ->
                        let points = [
                            for x in [lo.X;hi.X] do
                                for y in [lo.Y;hi.Y] do
                                    for z in [lo.Z;hi.Z] do
                                        yield Vec3.add part.Position (Rotation.apply part.Rotation (Vec3.create x y z))
                        ]
                        overlap low.X high.X (points |> List.minBy _.X).X (points |> List.maxBy _.X).X
                        * overlap low.Y high.Y (points |> List.minBy _.Y).Y (points |> List.maxBy _.Y).Y
                        * overlap low.Z high.Z (points |> List.minBy _.Z).Z (points |> List.maxBy _.Z).Z))

    let choicesWithPolicy policy (s: ScreenedMotion) (g: WalkerGenome) =
        let policy = WalkerBuilder.validate policy
        seq {
            for pivot in g.Connections |> List.sortBy _.Group do
                if (Catalog.get pivot.Placement.Part).Connector = Some FrictionlessPin then
                    let expected = members s.Program pivot
                    let originalRisk = clearanceRisk g expected pivot.Placement
                    for part in policy.PinOrder do
                        for reverse in [false; true] do
                            let old = pivot.Placement
                            let rotation =
                                if reverse then { old.Rotation with X = Vec3.scale -1.0 old.Rotation.X
                                                                    Z = Vec3.scale -1.0 old.Rotation.Z }
                                else old.Rotation
                            let lo, hi = Catalog.shaftRange part
                            let pin = {
                                old with
                                    Part = part
                                    Rotation = rotation
                                    Position = Vec3.subtract (midpoint old) (Vec3.scale ((lo+hi)/2.0) rotation.X)
                            }
                            if pin <> old && engaged g pin = expected && supportedStack g pin then
                                let candidate = replace g { pivot with Placement = pin }
                                match WalkerRealization.compile s candidate with
                                | Error _ -> ()
                                | Ok _ ->
                                    yield { Genome = candidate; Motif = motif old pin
                                            Pivot = pivot.Group; Members = expected
                                            ClearanceImprovement = originalRisk-clearanceRisk g expected pin
                                            AffectedBodies = []; LayerOffset = 0.0 }
        }
        |> Seq.distinctBy (fun choice -> WalkerRealization.json choice.Genome)

    let choices s g = choicesWithPolicy WalkerBuilder.defaultPolicy s g

    let private rank policy choices =
        if policy.ClearanceFirst then
            choices |> Seq.mapi (fun index choice -> index,choice)
            |> Seq.sortBy (fun (index,choice) -> -choice.ClearanceImprovement,index) |> Seq.map snd
        else choices

    let private motionLayouts (s: ScreenedMotion) (g: WalkerGenome) frames (baseline: WalkerClearanceReport) (implicated: Set<string>) measured =
        let targetedPlacements (genome: WalkerGenome) =
            (genome.Bodies |> List.collect _.Parts) @ genome.Mounts @ (genome.Connections |> List.map _.Placement)
            |> List.filter (fun part -> implicated.Contains part.Id) |> List.map (fun part -> part.Id,part) |> Map.ofList
        let before = targetedPlacements g
        let movesTarget (edit: WalkerPhysicalEdit) = targetedPlacements edit.Genome <> before
        let hardware = [
            let placements = WalkerLayout.receiverEdits s g |> List.filter movesTarget
            let facing (edit: WalkerPhysicalEdit) =
                edit.Genome.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id = "payload-receiver") |> _.Rotation.Z
            yield! placements |> List.groupBy facing |> List.collect (fun (_,edits) -> edits |> List.truncate 6)
            yield! WalkerLayout.batteryEdits s g |> List.filter movesTarget |> List.truncate 6
            yield! WalkerLayout.motorEdits s g
                   |> List.filter (fun edit -> edit.Program = s.Program && movesTarget edit) |> List.truncate 6
        ]
        hardware |> List.truncate 12 |> List.choose (fun edit ->
            match WalkerRealization.compile s edit.Genome with
            | Error _ -> None
            | Ok _ ->
                let after = WalkerClearance.evaluateFrames s frames edit.Genome baseline.RefinedIntervals
                if (if measured then after.Score > baseline.Score+1e-8 else after.Score >= baseline.Score-1e-8) then None
                else
                    Some { Genome = edit.Genome; Motif = "motion-clearance-"+edit.Operation; Pivot = ""; Members = []
                           ClearanceImprovement = baseline.Score-after.Score
                           AffectedBodies = []; LayerOffset = 0.0 })
        |> List.sortBy (fun choice -> -choice.ClearanceImprovement,WalkerRealization.json choice.Genome)

    let measuredLayouts s g parts limit =
        if limit < 1 || limit > 32 then invalidArg "limit" "Construction search permits one to 32 realizations."
        let frames,baseline = WalkerClearance.inspect s g
        if not baseline.EndpointRepeats || baseline.SamplingFailure.IsSome then [] else
        motionLayouts s g frames baseline (Set.ofList parts) true |> List.truncate limit

    let alternativesWithPolicy policy s g limit =
        let policy = WalkerBuilder.validate policy
        if limit < 1 || limit > 32 then invalidArg "limit" "Construction search permits one to 32 realizations."
        seq {
            yield { Genome = g; Motif = "inherited-assembly"; Pivot = ""; Members = []; ClearanceImprovement = 0.0
                    AffectedBodies = []; LayerOffset = 0.0 }
            if limit > 1 && policy.ClearanceFirst then
                let frames,baseline = WalkerClearance.inspect s g
                if baseline.SuspectPairs > 0 && baseline.EndpointRepeats && baseline.SamplingFailure.IsNone then
                    yield! motionLayouts s g frames baseline (baseline.Contacts |> List.collect _.Parts |> Set.ofList) false
            yield! choicesWithPolicy policy s g |> rank policy
        } |> Seq.truncate limit |> Seq.toList

    let alternatives s g limit = alternativesWithPolicy WalkerBuilder.defaultPolicy s g limit

    let describe (s: ScreenedMotion) (baseline: WalkerGenome) (g: WalkerGenome) =
        let movedBodies = g.Bodies |> List.filter (fun body -> baseline.Bodies |> List.exists (fun old ->
            old.Body = body.Body && old.Parts <> body.Parts)) |> List.map _.Body
        let changed = g.Connections |> List.filter (fun c ->
            baseline.Connections |> List.exists (fun b -> b.Group = c.Group && b <> c))
        match movedBodies,changed with
        | _::_,_ ->
            let frames,before = WalkerClearance.inspect s baseline
            let after = WalkerClearance.evaluateFrames s frames g before.RefinedIntervals
            { Genome = g; Motif = "motion-clearance-layout"; Pivot = ""; Members = []
              ClearanceImprovement = before.Score-after.Score
              AffectedBodies = movedBodies; LayerOffset = 0.0 }
        | [],[pivot] ->
            let before = baseline.Connections |> List.find (fun b -> b.Group = pivot.Group)
            { Genome = g; Motif = motif before.Placement pivot.Placement
              Pivot = pivot.Group; Members = members s.Program pivot
              ClearanceImprovement = clearanceRisk baseline (members s.Program pivot) before.Placement
                                     - clearanceRisk baseline (members s.Program pivot) pivot.Placement
              AffectedBodies = []; LayerOffset = 0.0 }
        | _ -> { Genome = g; Motif = "inherited-assembly"; Pivot = ""; Members = []; ClearanceImprovement = 0.0
                 AffectedBodies = []; LayerOffset = 0.0 }

    let repairsWithPolicy policy (s: ScreenedMotion) (g: WalkerGenome) (feedback: WalkerConstructionFeedback) limit =
        let policy = WalkerBuilder.validate policy
        if limit < 1 || limit > 32 then invalidArg "limit" "Construction search permits one to 32 realizations."
        if isNull (box feedback) || isNull (box feedback.Parts) || isNull (box feedback.Bodies) then
            invalidArg "feedback" "Construction feedback requires part and body lists."
        if not (List.contains feedback.Kind ["rigid-body-partition";"joint-port-engagement";"initial-self-interference";"motion-self-interference";"motion-clearance";"initial-foot-clearance"]) then
            invalidArg "feedback" "Unsupported construction feedback kind."
        if (feedback.Parts @ feedback.Bodies).IsEmpty
           || feedback.Parts @ feedback.Bodies |> List.exists System.String.IsNullOrWhiteSpace then
            invalidArg "feedback" "Construction feedback must identify affected parts or bodies."
        let targeted (body: WalkerBodyChoice) =
            List.contains body.Body feedback.Bodies
            || body.Parts |> List.exists (fun part -> List.contains part.Id feedback.Parts)
        let pinChoices =
            choicesWithPolicy policy s g
            |> Seq.filter (fun choice ->
                g.Connections |> List.exists (fun c ->
                    c.Group = choice.Pivot && List.contains c.Placement.Id feedback.Parts))
            |> rank policy
        let layers = seq {
            let protectedIds =
                s.Program.Bodies |> List.filter _.Grounded |> List.map _.Id
                |> Set.ofList |> Set.add s.Program.Motor.Body
                |> Set.union (s.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output]) |> Set.ofList)
            let chassis = s.Program.Bodies |> List.find _.Grounded
            let center = (g.Bodies |> List.find (fun b -> b.Body = chassis.Id)).Parts |> List.averageBy _.Position.Z
            for body in g.Bodies |> List.filter targeted |> List.sortBy _.Body do
                let pivots = g.Connections |> List.filter (fun c -> members s.Program c |> List.contains body.Body)
                let mounted = g.Mounts |> List.exists (fun pin -> body.Parts |> List.exists (WalkerStructural.engaged pin))
                if not (Set.contains body.Body protectedIds) && not mounted && not pivots.IsEmpty
                   && pivots |> List.forall (fun c -> (Catalog.get c.Placement.Part).Connector = Some FrictionlessPin) then
                    let outward = if (body.Parts |> List.averageBy _.Position.Z) >= center then 1.0 else -1.0
                    for distance in policy.LayerOffsets do
                        let offset = outward*float distance
                        let shift (p: WalkerPlacement) =
                            { p with Position = { p.Position with Z = p.Position.Z+offset } }
                        let moved = { body with Parts = body.Parts |> List.map shift }
                        let partIds = body.Parts |> List.map _.Id |> Set.ofList
                        let candidate = {
                            g with
                                Bodies = g.Bodies |> List.map (fun b -> if b.Body = body.Body then moved else b)
                                Feet = g.Feet |> List.map (WalkerFootChoice.shiftParts partIds offset)
                        }
                        let covered = pivots |> List.forall (fun pivot ->
                            let expected = members s.Program pivot
                            engaged candidate pivot.Placement = expected
                            && supportedStack candidate pivot.Placement)
                        if covered then
                            match WalkerRealization.compile s candidate with
                            | Error _ -> ()
                            | Ok _ ->
                                yield { Genome = candidate; Motif = "passive-link-layer-clearance"; Pivot = ""
                                        Members = []; ClearanceImprovement = 0.0
                                        AffectedBodies = [body.Body]; LayerOffset = offset }
        }
        let existing = policy.MotifOrder |> Seq.collect (function
            | "smooth-passive-pivot" -> pinChoices
            | "passive-link-layer-clearance" -> layers
            | id -> invalidArg "builder-policy" $"Unknown motif {id}.")
        let candidates =
            if feedback.Kind = "initial-foot-clearance" then
                initialGroundRepairs s g targeted |> Seq.ofList
            elif feedback.Kind = "motion-clearance" then
                let frames,baseline = WalkerClearance.inspect s g
                let targetedParts =
                    g.Bodies |> List.filter (fun body -> List.contains body.Body feedback.Bodies) |> List.collect _.Parts |> List.map _.Id
                    |> Set.ofList |> Set.union (Set.ofList feedback.Parts)
                let targetedContacts = baseline.Contacts |> List.filter (fun c -> c.Parts |> List.exists targetedParts.Contains)
                let targetedBaseline = { baseline with Contacts = targetedContacts }
                let layerChoices =
                    existing |> Seq.truncate 12 |> Seq.choose (fun choice ->
                        let after = WalkerClearance.evaluateFrames s frames choice.Genome baseline.RefinedIntervals
                        if after.Score >= baseline.Score-1e-8 then None
                        else Some { choice with Motif = "motion-clearance-"+choice.Motif
                                                ClearanceImprovement = baseline.Score-after.Score })
                if baseline.EndpointRepeats && baseline.SamplingFailure.IsNone then
                    Seq.append (motionLayouts s g frames targetedBaseline targetedParts false) layerChoices |> rank { policy with ClearanceFirst = true }
                else Seq.empty
            else existing
        candidates |> Seq.distinctBy (fun choice -> WalkerRealization.json choice.Genome)
                   |> Seq.truncate limit |> Seq.toList

    let repairs s g feedback limit = repairsWithPolicy WalkerBuilder.defaultPolicy s g feedback limit

    let writeWithPolicy policy directory choice =
        let report = {|
            constructionVersion = version; motif = choice.Motif; pivot = choice.Pivot
            catalogueVersion = WalkerBuilder.catalogueVersion; builderPolicyId = WalkerBuilder.policyId policy
            mechanismId = choice.Genome.MechanismId; intendedMembers = choice.Members
            clearanceRankingImprovement = choice.ClearanceImprovement
            affectedBodies = choice.AffectedBodies; layerOffsetLdu = choice.LayerOffset
            limits = ["Catalog-port coverage and compiler validation, not certified structural strength."
                      "Pivot ranking uses time-zero envelopes; motion-clearance motifs use advisory sampled capsule/box scores."
                      "Exact collision geometry, axial retention and load capacity require further validation."]
        |}
        File.WriteAllText(Path.Combine(directory, "construction.json"), Realization.json report+"\n")

    let write directory choice = writeWithPolicy WalkerBuilder.defaultPolicy directory choice

    let writeMotionEvidence directory s baseline genome =
        let frames,before = WalkerClearance.inspect s baseline
        let after = WalkerClearance.evaluateFrames s frames genome before.RefinedIntervals
        File.WriteAllText(Path.Combine(directory,"motion-clearance-repair.json"), Realization.json {|
            version = WalkerClearance.version; motionUnchanged = true
            commonSampledPoses = frames.Length; before = before; after = after
            scoreImprovement = before.Score-after.Score
            interpretation = "Approximate before/after ranking on identical prescribed poses, not validated collision clearance."
        |}+"\n")
