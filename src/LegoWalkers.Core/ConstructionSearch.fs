namespace LegoWalkers

open System

type ConstructionGoal = {
    Id: string
    Kind: string
    Subject: string
    Requires: string list
    External: bool
}

type ConstructionBond = {
    Connector: string
    PartA: string
    PositionA: Vec3
    AxisA: Vec3
    PartB: string
    PositionB: Vec3
    AxisB: Vec3
}

type ConstructionStep = {
    Goal: string
    Rule: string
    Choice: string
    Dependencies: string list
    Parts: string list
    Bodies: string list
    Cost: float
    Introduced: string list
    Bonds: ConstructionBond list
}

type ConstructionChoice = {
    Step: ConstructionStep
    Introduces: ConstructionGoal list
}

type ConstructionProof = {
    Scope: string
    Goals: ConstructionGoal list
    Steps: ConstructionStep list
}

type ConstructionFeedback = {
    Scope: string
    EvidenceSha256: string
    Reason: string
    Parts: string list
    Bodies: string list
    Exact: bool
}

type ConstructionNogood = {
    Feedback: ConstructionFeedback
    Bindings: (string * string) list
    Targets: string list
    Focus: string list
}

type ConstructionSchedule = {
    FamilyKinds: string list
    StatesPerSlice: int
    SliceVisited: int
    SliceLimit: int
    FamilyVisits: (string * int) list
}

type ConstructionDomain = {
    Bindings: (string * string) list
    Goal: string
    Choices: string list
}

type ConstructionAdvisory = { Bindings: (string * string) list; Reason: string }

type ConstructionConstraintVerdict =
    | Consistent
    | Inconsistent of bindings: (string * string) list
    | Inconclusive of reason: string

type ConstructionSearchState = {
    Scope: string
    Frontier: ConstructionProof list
    Deferred: ConstructionProof list
    Schedule: ConstructionSchedule option
    Domains: ConstructionDomain list
    ConstraintChecks: int
    ConstraintPruned: int
    ConstraintWarnings: string list
    Advisories: ConstructionAdvisory list
    Nogoods: ConstructionNogood list
    Refresh: ConstructionProof list
    SeenNodes: string list
    SeenWitnesses: string list
    Pending: ConstructionProof option
    Visited: int
    Pruned: int
    Discarded: int
    MaximumStates: int
    MaximumFrontier: int
    Status: string
    Rejections: GateDecision list
}

type ConstructionExpansion<'witness> =
    | Branch of goal: string * choices: ConstructionChoice list
    | GuidedBranch of goal: string * choices: ConstructionChoice list * preferred: string
    | AdvisedBranch of goal: string * choices: ConstructionChoice list * advisory: ConstructionAdvisory
    | Built of identity: string * witness: 'witness
    | Blocked of GateDecision
    | BlockedBy of gate: GateDecision * bindings: (string * string) list
    | BlockedByAdvised of gate: GateDecision * bindings: (string * string) list * advisory: ConstructionAdvisory

module ConstructionSearch =
    let version = "obligation-search-v12"
    let private require condition message = if not condition then invalidArg "construction-proof" message
    let private nonempty (text: string) = not(String.IsNullOrWhiteSpace text)
    let private distinct items = List.length items=(Set.ofList items).Count
    let nodeId (proof: ConstructionProof) =
        Export.sha256 (Realization.json (proof.Scope,proof.Steps |> List.map (fun s -> s.Goal,s.Choice)))

    let minimizeFiniteConflict maximumChecks bindings domains rejects =
        require (maximumChecks>=0 && not(List.isEmpty bindings) && distinct (List.map fst bindings))
                "Conflict reduction needs distinct bindings and a nonnegative check budget."
        let original = Map.ofList bindings
        require (distinct (List.map fst domains)
                 && (domains |> List.forall (fun (goal,choices) ->
                     original.ContainsKey goal && not(List.isEmpty choices) && distinct choices
                     && List.contains original[goal] choices)))
                "Every finite domain must contain its current binding and distinct alternatives."
        let mutable checks = 0
        let mutable interrupted = false
        let mutable remaining = bindings
        let mutable freed = []
        for goal,choices in domains do
            let proposed = freed @ [goal,choices]
            let budget = maximumChecks-checks
            let count = proposed |> List.fold (fun count (_,choices) ->
                if count>budget || choices.Length>budget/max 1 count then budget+1
                else count*choices.Length) 1
            if not interrupted && count<=budget then
                let assignments = proposed |> List.fold (fun assignments (goal,choices) ->
                    assignments |> List.collect (fun values -> choices |> List.map (fun choice -> Map.add goal choice values))) [Map.empty]
                let allRejected = assignments |> List.forall (fun values ->
                    let assignment = bindings |> List.map (fun (goal,choice) ->
                        goal,Map.tryFind goal values |> Option.defaultValue choice)
                    match rejects assignment with
                    | Some rejected -> checks<-checks+1;rejected
                    | None -> interrupted<-true;false)
                if allRejected then
                    freed <- proposed
                    remaining <- remaining |> List.filter (fun (id,_) -> id<>goal)
        remaining,checks

    let minimizeCoveredConflict maximumChecks bindings domains clauses =
        minimizeFiniteConflict maximumChecks bindings domains (fun assignment ->
            let values = Map.ofList assignment
            Some(clauses |> List.exists (fun clause ->
                not(List.isEmpty clause)
                && clause |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice))))

    let validate (proof: ConstructionProof) =
        require (nonempty proof.Scope && distinct (proof.Goals |> List.map _.Id)) "Scope and unique goal IDs are required."
        let goals = proof.Goals |> List.map (fun g -> g.Id,g) |> Map.ofList
        for goal in proof.Goals do
            require (nonempty goal.Id && nonempty goal.Kind && nonempty goal.Subject && distinct goal.Requires
                     && not(List.contains goal.Id goal.Requires)
                     && (goal.Requires |> List.forall goals.ContainsKey)) "Goals require valid, distinct prerequisite goals."
        let mutable remaining = proof.Goals
        let mutable resolved = Set.empty
        while not remaining.IsEmpty do
            let ready,waiting = remaining |> List.partition (fun g -> g.Requires |> List.forall resolved.Contains)
            require (not ready.IsEmpty) "The obligation dependency graph must be acyclic."
            resolved <- Set.union resolved (ready |> List.map _.Id |> Set.ofList)
            remaining <- waiting
        let mutable assigned = Set.empty
        let mutable produced = Set.empty
        let mutable producers = Map.empty
        for step in proof.Steps do
            require (goals.ContainsKey step.Goal && not(assigned.Contains step.Goal)) "Each step must discharge one existing open goal."
            require (nonempty step.Rule && nonempty step.Choice && Double.IsFinite step.Cost && step.Cost>=0.0)
                    "Production choices require finite nonnegative costs and stable rule/choice IDs."
            require (distinct step.Dependencies && (step.Dependencies |> List.forall assigned.Contains))
                    "Production dependencies must refer to earlier steps."
            require (goals[step.Goal].Requires |> List.forall (fun g -> List.contains g step.Dependencies))
                    "Every prerequisite must be recorded as a production dependency."
            require (distinct step.Parts && (step.Parts |> List.forall nonempty)
                     && (step.Parts |> List.forall (produced.Contains >> not))) "Every physical part requires one unique producing step."
            require (distinct step.Bodies && (step.Bodies |> List.forall nonempty)
                     && distinct step.Introduced && (step.Introduced |> List.forall goals.ContainsKey))
                    "Body provenance and introduced obligation IDs must be valid."
            for bond in step.Bonds do
                require (bond.PartA<>bond.PartB && bond.Connector<>bond.PartA && bond.Connector<>bond.PartB
                         && ([bond.Connector;bond.PartA;bond.PartB] |> List.forall produced.Contains))
                        "Connection witnesses must reference distinct, previously produced parts."
                require (([bond.PositionA;bond.PositionB;bond.AxisA;bond.AxisB] |> List.forall Vec3.finite)
                         && ([bond.AxisA;bond.AxisB] |> List.forall (fun axis -> abs(Vec3.length axis-1.0)<1e-8)))
                        "Connection witnesses require finite port positions and unit axes."
                require ([bond.Connector;bond.PartA;bond.PartB] |> List.forall (fun id -> List.contains producers[id] step.Dependencies))
                        "Connection witnesses must depend on every referenced part producer."
            assigned <- assigned.Add step.Goal
            produced <- Set.union produced (Set.ofList step.Parts)
            producers <- step.Parts |> List.fold (fun known id -> Map.add id step.Goal known) producers
        proof

    let openGoals proof =
        let proof = validate proof
        let assigned = proof.Steps |> List.map _.Goal |> Set.ofList
        proof.Goals |> List.filter (fun g -> not(assigned.Contains g.Id))

    let apply (proof: ConstructionProof) (choice: ConstructionChoice) =
        let proof = validate proof
        let goal = proof.Goals |> List.tryFind (fun g -> g.Id=choice.Step.Goal)
        require (goal.IsSome && not(goal.Value.External)) "Local productions cannot discharge external oracle obligations."
        let step = {choice.Step with Introduced=choice.Introduces |> List.map _.Id}
        let next = {proof with Goals=proof.Goals @ choice.Introduces;Steps=proof.Steps @ [step]}
        validate next

    let private forbidden (nogoods: ConstructionNogood list) proof =
        let values = proof.Steps |> List.map (fun s -> s.Goal,s.Choice) |> Map.ofList
        nogoods |> List.exists (fun n ->
            n.Feedback.Exact && n.Feedback.Scope=proof.Scope
            && (n.Bindings |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice)))

    let create scope goals maximumStates maximumFrontier =
        require (maximumStates>=1 && maximumStates<=16384 && maximumFrontier>=1 && maximumFrontier<=1024)
                "Search bounds must be within 1..16384 states and 1..1024 frontier nodes."
        let proof = validate {Scope=scope;Goals=goals;Steps=[]}
        {Scope=scope;Frontier=[proof];Deferred=[];Schedule=None
         Domains=[];ConstraintChecks=0;ConstraintPruned=0;ConstraintWarnings=[];Advisories=[]
         Nogoods=[];Refresh=[];SeenNodes=[];SeenWitnesses=[];Pending=None
         Visited=0;Pruned=0;Discarded=0;MaximumStates=maximumStates;MaximumFrontier=maximumFrontier
         Status="searching";Rejections=[]}

    let withFamilyScheduling familyKinds statesPerSlice state =
        require (not(List.isEmpty familyKinds) && distinct familyKinds
                 && List.forall nonempty familyKinds && statesPerSlice>=1)
                "Family scheduling needs distinct goal kinds and a positive state slice."
        require (state.Visited=0) "Configure scheduling before visiting search states."
        {state with Schedule=Some {FamilyKinds=familyKinds;StatesPerSlice=statesPerSlice
                                   SliceVisited=0;SliceLimit=statesPerSlice;FamilyVisits=[]}}

    let private bounded frontier state =
        let pending = frontier @ state.Deferred |> List.distinctBy nodeId
        {state with Frontier=pending |> List.truncate state.MaximumFrontier
                    Deferred=pending |> List.skip (min pending.Length state.MaximumFrontier)}

    let private familyPrefixes schedule (proof: ConstructionProof) =
        let kinds = proof.Goals |> List.map (fun goal -> goal.Id,goal.Kind) |> Map.ofList
        proof.Steps |> List.filter (fun step -> List.contains kinds[step.Goal] schedule.FamilyKinds)
        |> List.scan (fun prefix step -> prefix @ [step.Goal,step.Choice]) []
        |> List.tail |> List.map (Realization.json >> Export.sha256)

    let private selectSlice state =
        match state.Schedule with
        | Some schedule when schedule.SliceVisited>=schedule.SliceLimit ->
            let pending = state.Frontier @ state.Deferred
            let visits = Map.ofList schedule.FamilyVisits
            let score proof =
                let counts = familyPrefixes schedule proof |> List.map (fun key ->
                    Map.tryFind key visits |> Option.defaultValue 0)
                counts @ List.replicate (max 0 (schedule.FamilyKinds.Length-counts.Length)) 0
            let selected = pending |> List.minBy score
            let key = nodeId selected
            let rest = pending |> List.filter (fun proof -> nodeId proof<>key)
            let limit = min (schedule.StatesPerSlice*8) (schedule.SliceLimit*2)
            bounded (selected::rest) {state with Deferred=[];
                                                Schedule=Some {schedule with SliceVisited=0;SliceLimit=limit}}
        | _ when state.Frontier.IsEmpty -> bounded [] state
        | _ -> state

    let private selectAdvisory state =
        if state.Advisories.IsEmpty then state else
        let penalty (proof: ConstructionProof) =
            let values = proof.Steps |> List.map (fun step -> step.Goal,step.Choice) |> Map.ofList
            state.Advisories |> List.filter (fun advisory ->
                advisory.Bindings |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice)) |> List.length
        let pending = state.Frontier @ state.Deferred
        let selected = pending |> List.minBy penalty
        if nodeId selected=nodeId state.Frontier.Head then state else
        let key = nodeId selected
        bounded (selected::(pending |> List.filter (fun proof -> nodeId proof<>key))) {state with Deferred=[]}

    let private recordFamilyVisit proof state =
        match state.Schedule with
        | None -> state
        | Some schedule ->
            let visits =
                familyPrefixes schedule proof |> List.fold (fun counts key ->
                    Map.add key (1+(Map.tryFind key counts |> Option.defaultValue 0)) counts)
                    (Map.ofList schedule.FamilyVisits)
            let schedule = {schedule with SliceVisited=schedule.SliceVisited+1;FamilyVisits=Map.toList visits}
            {state with Schedule=Some schedule}

    let reject (feedback: ConstructionFeedback) state =
        require (feedback.Scope=state.Scope && nonempty feedback.EvidenceSha256 && nonempty feedback.Reason)
                "Oracle feedback must be bound to this exact search/oracle scope and immutable evidence."
        let proof = state.Pending |> Option.defaultWith (fun () -> invalidArg "feedback" "No candidate is awaiting an oracle verdict.")
        require (openGoals proof |> List.forall _.External) "Only complete local witnesses can receive oracle feedback."
        let partContributors = proof.Steps |> List.filter (fun s ->
            s.Parts |> List.exists (fun p -> List.contains p feedback.Parts)
            || s.Bonds |> List.exists (fun b ->
                [b.Connector;b.PartA;b.PartB] |> List.exists (fun p -> List.contains p feedback.Parts)))
        let bodyContributors = proof.Steps |> List.filter (fun s ->
            s.Bodies |> List.exists (fun b -> List.contains b feedback.Bodies)) |> List.map _.Goal |> Set.ofList
        let directParts = partContributors |> List.map _.Goal |> Set.ofList
        let relevant =
            if List.contains feedback.Reason ["rigid-bond-engagement";"joint-port-engagement"] && not directParts.IsEmpty then directParts
            // A partition mismatch names displaced members, not necessarily the connector that displaced them.
            else Set.union directParts bodyContributors
        let mutable dependencies = Set.union relevant bodyContributors
        for step in List.rev proof.Steps do
            if dependencies.Contains step.Goal then dependencies <- Set.union dependencies (Set.ofList step.Dependencies)
        let targets =
            if dependencies.IsEmpty then proof.Steps |> List.map _.Goal
            else proof.Steps |> List.filter (fun s -> dependencies.Contains s.Goal) |> List.map _.Goal
        // Without a separately validated causal lemma, only the exact failed witness is a hard nogood.
        let nogood = {Feedback=feedback;Bindings=proof.Steps |> List.map (fun s -> s.Goal,s.Choice);Targets=targets
                      Focus=proof.Steps |> List.filter (fun s -> relevant.Contains s.Goal) |> List.map _.Goal}
        {state with Pending=None;Domains=[];Nogoods=state.Nogoods @ [nogood]
                    Refresh=state.Refresh @ [proof];Status="searching"}

    let acceptVerified evidenceSha256 verifiedGoals state =
        require (nonempty evidenceSha256) "An oracle acceptance requires immutable evidence."
        let proof = state.Pending |> Option.defaultWith (fun () -> invalidArg "oracle" "No candidate is awaiting an oracle verdict.")
        let local = openGoals proof |> List.filter (fun g -> not g.External)
        require local.IsEmpty "An incomplete construction cannot be accepted."
        require (distinct verifiedGoals && Set.ofList verifiedGoals=(openGoals proof |> List.map _.Id |> Set.ofList))
                "Oracle acceptance must explicitly verify every open external obligation."
        let mutable certified = proof
        while not (openGoals certified).IsEmpty do
            let assigned = certified.Steps |> List.map _.Goal |> Set.ofList
            let goal = openGoals certified |> List.find (fun g -> g.Requires |> List.forall assigned.Contains)
            let dependencies = certified.Steps |> List.map _.Goal
            let step =
                {Goal=goal.Id;Rule="external-oracle";Choice=evidenceSha256;Dependencies=dependencies
                 Parts=[];Bodies=[];Cost=0.0;Introduced=[];Bonds=[]}
            certified <- validate {certified with Steps=certified.Steps @ [step]}
        {state with Pending=None;Status="oracle-accepted"},certified

    let accept evidenceSha256 state =
        let proof = state.Pending |> Option.defaultWith (fun () -> invalidArg "oracle" "No candidate is awaiting an oracle verdict.")
        require (openGoals proof |> List.forall (fun g ->
                    not(List.contains g.Kind ["exact-rigid-connectivity";"exact-joint-preservation"])))
                "Connectivity and joint-preservation obligations require explicit verified-goal evidence."
        acceptVerified evidenceSha256 (openGoals proof |> List.map _.Id) state

    let private withDomain (proof: ConstructionProof) goal (choices: ConstructionChoice list) state =
        require (openGoals proof |> List.exists (fun candidate -> candidate.Id=goal)) "Branch must discharge an open goal."
        require (choices |> List.forall (fun choice -> choice.Step.Goal=goal)) "Branch choices must discharge the requested goal."
        let domain = {Bindings=proof.Steps |> List.map (fun step -> step.Goal,step.Choice)
                      Goal=goal;Choices=choices |> List.map (fun choice -> choice.Step.Choice) |> List.distinct}
        {state with Domains=List.distinct(state.Domains @ [domain])}

    let nextObservedWithConstraints check observe (expand: ConstructionProof -> ConstructionNogood list -> ConstructionExpansion<'witness>) state =
        require state.Pending.IsNone "Resolve the pending oracle verdict before continuing search."
        require (state.Status<>"oracle-accepted") "Accepted searches are terminal."
        require (state.MaximumStates>=1 && state.MaximumStates<=16384
                 && state.MaximumFrontier>=1 && state.MaximumFrontier<=1024
                 && state.Visited>=0 && state.Pruned>=0 && state.Discarded>=0
                 && state.ConstraintChecks>=0 && state.ConstraintPruned>=0 && state.ConstraintPruned<=state.Pruned)
                 "Checkpoint search accounting or bounds are invalid."
        state.Schedule |> Option.iter (fun schedule ->
            require (schedule.StatesPerSlice>=1 && schedule.SliceVisited>=0
                     && schedule.SliceLimit>=schedule.StatesPerSlice && schedule.SliceLimit<=schedule.StatesPerSlice*8
                     && not schedule.FamilyKinds.IsEmpty && distinct schedule.FamilyKinds
                     && List.forall nonempty schedule.FamilyKinds
                     && distinct (schedule.FamilyVisits |> List.map fst)
                     && (schedule.FamilyVisits |> List.forall (fun (key,count) -> nonempty key && count>=0)))
                    "Checkpoint family scheduling is invalid.")
        let mutable state = state
        let feedback = state.Nogoods
        let mutable refreshed = []
        for rejected in state.Refresh do
            require (rejected.Scope=state.Scope) "Refresh proof scope differs from the search."
            let nogood = feedback |> List.findBack (fun n -> n.Bindings=(rejected.Steps |> List.map (fun s -> s.Goal,s.Choice)))
            for index,step in List.indexed rejected.Steps do
                if List.contains step.Goal nogood.Targets && state.Visited<state.MaximumStates then
                    let steps = rejected.Steps |> List.take index
                    let introduced = rejected.Steps |> List.collect _.Introduced |> Set.ofList
                    let retained = steps |> List.collect _.Introduced |> Set.ofList
                    let goals = rejected.Goals |> List.filter (fun g ->
                        not(introduced.Contains g.Id) || retained.Contains g.Id)
                    let prefix = {rejected with Steps=steps;Goals=goals}
                    state <- {state with Visited=state.Visited+1}
                    match expand prefix feedback with
                    | Branch(goal,choices) | GuidedBranch(goal,choices,_) | AdvisedBranch(goal,choices,_) when goal=step.Goal ->
                        state <- withDomain prefix goal choices state
                        for choice in choices |> List.sortBy (fun c -> c.Step.Cost,c.Step.Choice) do
                            if choice.Step.Choice<>step.Choice then
                                let child = apply prefix choice
                                if not(List.contains (nodeId child) state.SeenNodes) then
                                    let priority = if List.contains step.Goal nogood.Focus then 0 else 1
                                    refreshed <- refreshed @ [priority,child]
                    | _ -> ()
        let refreshed = refreshed |> List.sortBy (fun (priority,p) ->
            let changed = List.last p.Steps
            priority,changed.Cost,changed.Goal,nodeId p) |> List.map snd
        state <- bounded (refreshed @ state.Frontier) {state with Refresh=[]}
        let mutable result = None
        while result.IsNone && not(state.Frontier.IsEmpty && state.Deferred.IsEmpty) && state.Visited<state.MaximumStates do
            state <- selectSlice state
            state <- selectAdvisory state
            let proof = state.Frontier.Head
            require (proof.Scope=state.Scope) "Frontier proof scope differs from the search."
            state <- {state with Frontier=state.Frontier.Tail}
            let key = nodeId proof
            let alreadyPruned = forbidden state.Nogoods proof || List.contains key state.SeenNodes
            let verdict =
                match check with
                | Some check when not alreadyPruned ->
                    state <- {state with ConstraintChecks=state.ConstraintChecks+1}
                    check state proof
                | _ -> Consistent
            match verdict with
            | Inconclusive reason ->
                require (nonempty reason) "An inconclusive constraint check requires an explicit reason."
                state <- {state with ConstraintWarnings=List.distinct(state.ConstraintWarnings @ [reason])}
            | _ -> ()
            if alreadyPruned then
                state <- {state with Pruned=state.Pruned+1}
            elif (match verdict with Inconsistent _ -> true | _ -> false) then
                let bindings = match verdict with Inconsistent bindings -> bindings | _ -> []
                let values = proof.Steps |> List.map (fun step -> step.Goal,step.Choice) |> Map.ofList
                require (distinct bindings && (bindings |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice)))
                        "Constraint explanations must name assumptions in the current proof."
                let gate = {Gate="construction-constraints";Outcome="reject";Code="bounded-domain-inconsistent"
                            Diagnostics=["Registered finite domains and exact scoped conflicts exclude this partial proof; not a proof of physical impossibility."]}
                state <- {state with Pruned=state.Pruned+1;ConstraintPruned=state.ConstraintPruned+1
                                     Rejections=state.Rejections @ [gate]}
                observe state proof (Some(if bindings.IsEmpty then Blocked gate else BlockedBy(gate,bindings)))
            else
                state <- {state with SeenNodes=key::state.SeenNodes;Visited=state.Visited+1}
                state <- recordFamilyVisit proof state
                observe state proof None
                let expansion = expand proof state.Nogoods
                observe state proof (Some expansion)
                let recordAdvice advisory =
                    let values = proof.Steps |> List.map (fun step -> step.Goal,step.Choice) |> Map.ofList
                    require (nonempty advisory.Reason && not advisory.Bindings.IsEmpty && distinct advisory.Bindings
                             && (advisory.Bindings |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice)))
                            "Scheduling advice must explain selected proof bindings."
                    state <- {state with Advisories=List.distinct(state.Advisories @ [advisory])}
                let expansion =
                    match expansion with
                    | BlockedByAdvised(gate,bindings,advisory) -> recordAdvice advisory;BlockedBy(gate,bindings)
                    | other -> other
                match expansion with
                | Blocked gate ->
                    require (gate.Outcome="reject") "A blocked production must return an explicit rejection."
                    state <- {state with Rejections=state.Rejections @ [gate]}
                | BlockedBy(gate,bindings) ->
                    require (gate.Outcome="reject" && not bindings.IsEmpty && distinct bindings)
                            "A local conflict requires rejection and nonempty distinct causal bindings."
                    let values = proof.Steps |> List.map (fun s -> s.Goal,s.Choice) |> Map.ofList
                    require (bindings |> List.forall (fun (goal,choice) -> Map.tryFind goal values=Some choice))
                            "Local conflict bindings must name decisions in the current proof."
                    let feedback =
                        {Scope=proof.Scope;EvidenceSha256=Export.sha256(Realization.json (gate,bindings))
                         Reason=gate.Code;Parts=gate.Diagnostics;Bodies=[];Exact=true}
                    let nogood = {Feedback=feedback;Bindings=bindings;Targets=bindings |> List.map fst;Focus=[]}
                    state <- {state with Nogoods=(state.Nogoods @ [nogood] |> List.distinct)
                                         Rejections=state.Rejections @ [gate]}
                | Branch(goal,choices) | GuidedBranch(goal,choices,_) | AdvisedBranch(goal,choices,_) ->
                    state <- withDomain proof goal choices state
                    match expansion with
                    | AdvisedBranch(_,_,advisory) -> recordAdvice advisory
                    | _ -> ()
                    let preferred =
                        match expansion with
                        | GuidedBranch(_,_,value) ->
                            require (choices |> List.exists (fun choice -> choice.Step.Choice=value)) "A guided branch must prefer an available choice."
                            let schedule = state.Schedule |> Option.map (fun schedule ->
                                {schedule with SliceLimit=schedule.StatesPerSlice*8})
                            state <- {state with Schedule=schedule}
                            Some value
                        | _ -> None
                    if choices.IsEmpty then
                        let gate = {Gate="construction-search";Outcome="reject";Code="no-applicable-production";Diagnostics=[goal]}
                        state <- {state with Rejections=state.Rejections @ [gate]}
                    else
                        let children = choices |> List.sortBy (fun c ->
                            (preferred |> Option.exists (fun value -> value<>c.Step.Choice)),c.Step.Cost,c.Step.Choice)
                                               |> List.map (apply proof)
                        state <- bounded (children @ state.Frontier) state
                | Built(identity,witness) ->
                    require (nonempty identity && (openGoals proof |> List.forall _.External))
                            "A physical witness needs an identity and no unresolved local obligations."
                    if List.contains identity state.SeenWitnesses then state <- {state with Pruned=state.Pruned+1}
                    else
                        state <- {state with Pending=Some proof;SeenWitnesses=identity::state.SeenWitnesses;Status="awaiting-oracle"}
                        result <- Some witness
                | BlockedByAdvised _ -> invalidOp "Scheduling advice was not normalized."
        if result.IsNone then
            let status =
                if state.Frontier.IsEmpty && state.Deferred.IsEmpty then
                    if state.Discarded>0 then "frontier-bound-exhausted" else "productions-exhausted"
                else "state-bound-reached"
            state <- {state with Status=status}
        state,result

    let nextObserved observe expand state = nextObservedWithConstraints None observe expand state
    let next expand state = nextObserved (fun _ _ _ -> ()) expand state
