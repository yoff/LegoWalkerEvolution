namespace LegoWalkers

open System
open System.Collections.Generic
open System.IO
open System.Text.Json
open System.Text.Json.Serialization

type WalkerGrammarConflicts = { Pairs: string list list }
type WalkerGrammarStep = { Production: string; Subject: string; Choice: string }
type WalkerGrammarChoice = { Construction: WalkerConstructionChoice; Derivation: WalkerGrammarStep list }
type WalkerGrammarSearch = {
    Choices: WalkerGrammarChoice list
    VisitedStates: int
    CompleteAssignments: int
    UnsupportedStacks: int
    UnchangedConflicts: int
    CompilerRejections: GateDecision list
    BoundReached: bool
}

module WalkerConnectionGrammar =
    let version = "connection-link-grammar-v2"
    let maximumStates = 16384
    let maximumLayouts = 256
    let maximumStacksPerLayout = 32
    let maximumMovedLinks = 2
    let productions = [
        "Assembly -> LayeredLinks JointStacks"
        "LayeredLinks(link :: rest) -> LinkLayer(link, depth) LayeredLinks(rest)"
        "LinkLayer -> InheritedLayer | Outward(10) | Inward(10) | Outward(20) | Inward(20)"
        "JointStacks(pivot :: rest) -> RevoluteConnection(pivot) JointStacks(rest)"
        "RevoluteConnection -> CompatibleSockets Connector AxialPlacement"
        "Connector -> SmoothPin | StockAxle"
        "SmoothPin -> 32002.dat | 3673.dat | 32556.dat"
        "StockAxle -> " + String.concat " | " Catalog.axleParts
        "AxialPlacement -> Inherited | CenteredCoverage | CoverageBoundary"
    ]

    type private Symbol = LayeredLinks of string list | JointStacks of WalkerPivotChoice list

    let readConflicts path =
        let options = JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                                            UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
                                            RespectRequiredConstructorParameters = true)
        JsonSerializer.Deserialize<WalkerGrammarConflicts>(File.ReadAllText path, options)

    let private placements (g: WalkerGenome) =
        (g.Bodies |> List.collect _.Parts) @ (g.Connections |> List.map _.Placement) @ g.Mounts
        |> List.map (fun p -> p.Id,p) |> Map.ofList

    let private sameGeometry (a: WalkerPlacement) (b: WalkerPlacement) =
        a.Part = b.Part && Vec3.distance a.Position b.Position < 1e-6
        && Vec3.distance a.Rotation.X b.Rotation.X < 1e-6
        && Vec3.distance a.Rotation.Y b.Rotation.Y < 1e-6
        && Vec3.distance a.Rotation.Z b.Rotation.Z < 1e-6

    let private pinVariants program (baseline: WalkerGenome) (g: WalkerGenome) (pivot: WalkerPivotChoice) =
        let required =
            baseline.Bodies |> List.collect _.Parts
            |> List.filter (WalkerStructural.engaged pivot.Placement)
            |> List.map _.Id
        let byId = placements g
        let axle = (Catalog.get pivot.Placement.Part).Connector = Some Axle
        let supported choice =
            WalkerConstruction.pivotSupported program g choice
            && (not axle || required |> List.forall (fun id -> WalkerStructural.engaged choice.Placement byId[id]))
        let stocks = if axle then Catalog.axleParts else WalkerBuilder.defaultPolicy.PinOrder
        let walls axis =
            required |> List.map (fun id ->
                let part = byId[id]
                if axle then
                    let points = WalkerConstruction.axleWallPoints pivot.Placement part
                    if points.IsEmpty then None else Some (points |> List.map (fun point -> Vec3.dot point axis))
                else
                    WalkerConstruction.socketEnvelope part.Part
                    |> Option.map (fun (lo,hi) -> [
                        for x in [lo.X;hi.X] do
                            for y in [lo.Y;hi.Y] do
                                for z in [lo.Z;hi.Z] ->
                                    Vec3.dot (Vec3.add part.Position (Rotation.apply part.Rotation (Vec3.create x y z))) axis
                    ]))
        seq {
            if supported pivot then yield pivot
            for stock in stocks do
                for reverse in [false;true] do
                    let old = pivot.Placement
                    let rotation =
                        if reverse then { old.Rotation with X = Vec3.scale -1.0 old.Rotation.X
                                                            Z = Vec3.scale -1.0 old.Rotation.Z }
                        else old.Rotation
                    let envelopes = walls rotation.X
                    if not envelopes.IsEmpty && envelopes |> List.forall Option.isSome then
                        let points = envelopes |> List.collect Option.get
                        let lo,hi = Catalog.shaftRange stock
                        let minimum,maximum = List.max points-hi,List.min points-lo
                        if minimum <= maximum+1e-6 then
                            let original = Vec3.dot old.Position rotation.X
                            let clamped = max minimum (min maximum original)
                            for along in [clamped; (minimum+maximum)/2.0; minimum; maximum] |> List.distinct do
                                let pin = { old with Part = stock; Rotation = rotation
                                                     Position = Vec3.add old.Position (Vec3.scale (along-original) rotation.X) }
                                let choice = { pivot with Placement = pin }
                                if supported choice then yield choice
        }
        |> Seq.distinct
        |> Seq.sortBy (fun p ->
            let risk = WalkerConstruction.clearanceRisk g (WalkerConstruction.members program p) p.Placement
            risk, (if p = pivot then 0 else 1), p.Placement.Part, p.Placement.Position.Z)
        |> Seq.toList

    let derive (s: ScreenedMotion) (g: WalkerGenome) (conflicts: WalkerGrammarConflicts) limit =
        if limit < 1 || limit > 32 then invalidArg "limit" "Grammar search permits one to 32 exported realizations."
        let before = placements g
        if isNull (box conflicts) || isNull (box conflicts.Pairs)
           || conflicts.Pairs |> List.exists (fun pair ->
               isNull (box pair) || pair.Length <> 2 || pair[0] = pair[1]
               || pair |> List.exists (fun id -> String.IsNullOrWhiteSpace id || not (Map.containsKey id before))) then
            invalidArg "conflicts" "Expected pairs of distinct existing part IDs."
        let implicated = conflicts.Pairs |> List.collect id |> Set.ofList
        let protectedBodies =
            s.Program.Bodies |> List.filter _.Grounded |> List.map _.Id |> Set.ofList
            |> Set.add s.Program.Motor.Body
            |> Set.union (s.Program.Transmissions |> List.collect (fun t -> [t.Input;t.Output]) |> Set.ofList)
        let implicatedPivots = g.Connections |> List.filter (fun c -> Set.contains c.Placement.Id implicated)
        let relatedBodies = implicatedPivots |> List.collect (WalkerConstruction.members s.Program) |> Set.ofList
        let movable =
            g.Bodies |> List.filter (fun body ->
                let pivots = g.Connections |> List.filter (WalkerConstruction.members s.Program >> List.contains body.Body)
                not (Set.contains body.Body protectedBodies)
                && (Set.contains body.Body relatedBodies || body.Parts |> List.exists (fun p -> Set.contains p.Id implicated))
                && not pivots.IsEmpty
                && pivots |> List.forall (fun p ->
                    (Catalog.get p.Placement.Part).Connector = Some FrictionlessPin
                    && abs p.Placement.Rotation.X.Z > 1.0-1e-6)
                && not (g.Mounts |> List.exists (fun pin -> body.Parts |> List.exists (WalkerStructural.engaged pin))))
            |> List.sortBy _.Body |> List.map _.Body
        let center =
            let ground = s.Program.Bodies |> List.find _.Grounded
            (g.Bodies |> List.find (fun b -> b.Body = ground.Id)).Parts |> List.averageBy _.Position.Z
        let mutable visited, complete, unsupported, unchanged, layouts = 0,0,0,0,0
        let mutable bounded = false
        let errors = ResizeArray<GateDecision>()
        let found = ResizeArray<WalkerGrammarChoice>()
        let seen = HashSet<string>()
        let rec expand symbol (genome: WalkerGenome) steps moved stop emit =
            if stop () || visited >= maximumStates then bounded <- true
            else
                visited <- visited+1
                match symbol with
                | LayeredLinks (bodyId::rest) ->
                    let body = genome.Bodies |> List.find (fun b -> b.Body = bodyId)
                    let outward = if body.Parts |> List.averageBy _.Position.Z >= center then 1.0 else -1.0
                    for distance in [0;10;-10;20;-20] do
                        if distance = 0 || moved < maximumMovedLinks then
                            let delta = outward*float distance
                            let ids = body.Parts |> List.map _.Id |> Set.ofList
                            let candidate = {
                                genome with
                                    Bodies = genome.Bodies |> List.map (fun b ->
                                        if b.Body <> bodyId then b else
                                        { b with
                                            Parts = b.Parts |> List.map (fun p ->
                                                { p with Position = { p.Position with Z = p.Position.Z+delta } }) })
                                    Feet = genome.Feet |> List.map (WalkerFootChoice.shiftParts ids delta)
                            }
                            let step = { Production = "LayeredLinks -> LinkLayer LayeredLinks"; Subject = bodyId
                                         Choice = $"depth-offset:{delta}" }
                            expand (LayeredLinks rest) candidate (step::steps) (moved+(if distance = 0 then 0 else 1)) stop emit
                | LayeredLinks [] ->
                    if layouts >= maximumLayouts then bounded <- true
                    else
                        layouts <- layouts+1
                        let movedIds =
                            List.zip g.Bodies genome.Bodies |> List.choose (fun (a,b) -> if a = b then None else Some a.Body)
                            |> Set.ofList
                        let joints =
                            genome.Connections |> List.filter (fun p ->
                                ((Catalog.get p.Placement.Part).Connector = Some FrictionlessPin
                                 && (Set.contains p.Placement.Id implicated
                                     || WalkerConstruction.members s.Program p |> List.exists (fun id -> Set.contains id movedIds)))
                                || ((Catalog.get p.Placement.Part).Connector = Some Axle
                                    && Set.contains p.Placement.Id implicated))
                            |> List.sortBy _.Group
                        let mutable stacks = 0
                        expand (JointStacks joints) genome steps moved (fun () -> stacks >= maximumStacksPerLayout) (fun candidate trace ->
                            if stacks < maximumStacksPerLayout then
                                stacks <- stacks+1
                                emit candidate trace)
                | JointStacks (pivot::rest) ->
                    let variants = pinVariants s.Program g genome pivot
                    if variants.IsEmpty then unsupported <- unsupported+1
                    for choice in variants do
                        let candidate = {
                            genome with
                                Connections = genome.Connections |> List.map (fun p ->
                                    if p.Group = pivot.Group then choice else p)
                        }
                        let step = { Production = "JointStacks -> RevoluteConnection JointStacks"; Subject = pivot.Group
                                     Choice = WalkerRealization.json choice.Placement }
                        expand (JointStacks rest) candidate (step::steps) moved stop emit
                | JointStacks [] -> emit genome (List.rev steps)
        expand (LayeredLinks movable) g [] 0 (fun () -> false) (fun genome trace ->
            complete <- complete+1
            let after = placements genome
            if conflicts.Pairs |> List.exists (List.forall (fun id -> sameGeometry before[id] after[id])) then
                unchanged <- unchanged+1
            elif seen.Add (WalkerRealization.json genome) then
                match WalkerRealization.compile s genome with
                | Error gate -> errors.Add gate
                | Ok _ ->
                    found.Add {
                        Construction = { Genome = genome; Motif = version; Pivot = ""; Members = []
                                         ClearanceImprovement = 0.0; LayerOffset = 0.0
                                         AffectedBodies = List.zip g.Bodies genome.Bodies |> List.choose (fun (a,b) ->
                                             if a = b then None else Some a.Body) }
                        Derivation = { Production = "Assembly -> LayeredLinks JointStacks"; Subject = s.MechanismId
                                       Choice = "preserve-planar-motion" } :: trace
                    })
        let score (choice: WalkerGrammarChoice) =
            let genome = choice.Construction.Genome
            let pinRisk =
                genome.Connections |> List.sumBy (fun p ->
                    if List.contains (Catalog.get p.Placement.Part).Connector [Some FrictionlessPin;Some Axle] then
                        WalkerConstruction.clearanceRisk genome (WalkerConstruction.members s.Program p) p.Placement
                    else 0.0)
            let axleSpan =
                genome.Connections |> List.sumBy (fun p ->
                    if (Catalog.get p.Placement.Part).Connector = Some Axle then
                        let lo,hi = Catalog.shaftRange p.Placement.Part
                        hi-lo
                    else 0.0)
            choice.Construction.AffectedBodies.Length, pinRisk,
            axleSpan,
            (List.zip g.Connections genome.Connections |> List.filter (fun (a,b) -> a <> b) |> List.length)
        let byLayout =
            found |> Seq.groupBy (fun choice -> WalkerRealization.json choice.Construction.Genome.Bodies)
            |> Seq.map (fun (_,choices) -> choices |> Seq.sortBy score |> Seq.toList) |> Seq.toList
        let diverse =
            Seq.append (byLayout |> Seq.map List.head |> Seq.sortBy score)
                       (byLayout |> Seq.collect List.tail |> Seq.sortBy score)
        { Choices = diverse |> Seq.truncate limit |> Seq.toList
          VisitedStates = visited; CompleteAssignments = complete; UnsupportedStacks = unsupported
          UnchangedConflicts = unchanged; CompilerRejections = List.ofSeq errors; BoundReached = bounded }

    let writeDerivation directory choice =
        File.WriteAllText(Path.Combine(directory,"grammar.json"),
            Realization.json {| grammarVersion = version; productions = productions; derivation = choice.Derivation
                                limitation = "Coverage-checked derivation, not a collision or retention certificate." |}+"\n")

    let describeSearch search = {|
        visitedStates = search.VisitedStates; completeAssignments = search.CompleteAssignments
        unsupportedStacks = search.UnsupportedStacks; unchangedConflictAssignments = search.UnchangedConflicts
        compilerRejections = search.CompilerRejections; boundReached = search.BoundReached
        maximumStates = maximumStates; maximumLayouts = maximumLayouts
        maximumStacksPerLayout = maximumStacksPerLayout; maximumMovedLinks = maximumMovedLinks
    |}

    let writeBatch directory (s: ScreenedMotion) (baseline: RealizedWalker) conflicts search =
        if Directory.Exists directory && not (Directory.EnumerateFileSystemEntries directory |> Seq.isEmpty) then
            invalidArg "output" "Output directory must be empty."
        Directory.CreateDirectory directory |> ignore
        let generated, rejected = ResizeArray<_>(),ResizeArray<_>()
        let seen = HashSet<string>()
        for index,choice in search.Choices |> List.indexed do
            let robot =
                WalkerRealization.compile s choice.Construction.Genome
                |> Result.defaultWith (fun gate -> invalidOp (Realization.json gate))
            let geometry = Realization.screenAssemblyGeometry s robot.Assembly
            if geometry.Outcome = "reject" then rejected.Add {| attempt = index+1; decision = geometry |}
            elif seen.Add robot.AssemblyId then
                let name = $"candidate-{index+1:D3}"
                let destination = WalkerClearance.writeRealization (Path.Combine(directory,name)) s robot
                writeDerivation destination choice
                generated.Add {| directory = name; assemblyId = robot.AssemblyId; genotypeId = robot.GenotypeId |}
        let summary = {|
            schemaVersion = 1; gateVersion = Realization.gateVersion; mechanismId = s.MechanismId
            parentGenotypeId = baseline.GenotypeId; grammarVersion = version; operation = "experimental-connection-grammar"
            gates = s.Decisions; generated = generated.ToArray(); rejected = rejected.ToArray()
            counts = {| proposals = 1; realizationAttempts = search.Choices.Length; rejected = rejected.Count
                        duplicates = search.Choices.Length-generated.Count-rejected.Count
                        exported = generated.Count; dynamicTrials = 0 |}
            search = describeSearch search
            conflicts = conflicts; ranking = "Connector envelope overlap heuristic only; no hard broad-phase rejection."
            promotion = "Used for screening-guided initial-overlap repairs; standalone export does not authorize dynamics."
        |}
        File.WriteAllText(Path.Combine(directory,"candidates.json"),Realization.json summary+"\n")
        summary
