open System
open LegoWalkers

// A symbolic Trudge-shaped case: conflicts below are defined, not measured geometry.
let scope = "symbolic-trudge-support-v1"
let goal id dependencies =
    {Id=id;Kind=id;Subject=id;Requires=dependencies;External=false}
let goals = [goal "scaffold" [];goal "axle-shaft-4" ["scaffold"];goal "motor-bearing-support" ["scaffold"]]
let initial = ConstructionSearch.create scope goals 64 16
let step id value dependencies =
    {Step={Goal=id;Rule="symbolic-example";Choice=value;Dependencies=dependencies
           Parts=[];Bodies=[];Cost=0.0;Introduced=[];Bonds=[]};Introduces=[]}
let inherited = initial.Frontier.Head
                |> fun proof -> ConstructionSearch.apply proof (step "scaffold" "inherited" [])
let assumed = inherited
              |> fun proof -> ConstructionSearch.apply proof (step "axle-shaft-4" "9L" ["scaffold"])
let domain routes = {Bindings=["scaffold","inherited"];Goal="motor-bearing-support";Choices=routes}
let conflict route =
    {Feedback={Scope=scope;EvidenceSha256=Export.sha256 "defined-symbolic-conflict";Exact=true
               Reason="symbolic-example-only";Parts=[];Bodies=[]}
     Bindings=["scaffold","inherited";"motor-bearing-support",route];Targets=[];Focus=[]}
let constrained =
    {initial with
        Domains=[{Bindings=[];Goal="scaffold";Choices=["inherited";"shifted"]};domain ["route-a";"route-b"]]
        Nogoods=[conflict "route-a";conflict "route-b"]}
let report =
    use solver = new Z3ConstructionSolver(fsi.CommandLineArgs[1],scope)
    let core =
        match solver.Check(constrained,assumed) with
        | Inconsistent bindings when bindings=["scaffold","inherited"] -> bindings
        | verdict -> failwithf "Expected an exhausted support domain to exclude only the scaffold assumption, got %A" verdict
    let unseen = {constrained with Domains=[constrained.Domains.Head;domain ["route-a";"route-b";"route-c"]]}
    if solver.Check(unseen,assumed)<>Consistent then failwith "An untested support route must remain feasible."
    let repaired = {constrained with Domains=[];Nogoods=[]}
    if solver.Check(repaired,assumed)<>Consistent then failwith "Removed repair-sensitive domains must not survive in the solver."
    // Allocate actual catalog sockets without attempting a complete robot.
    let frame = {Id="example-frame";Part="32523.dat";Color=7;Position=Vec3.zero;Rotation=Rotation.beam (Vec3.create 1 0 0)}
    let ports = (Catalog.get frame.Part).Ports |> List.filter (fun port -> port.Kind=RoundHole)
                |> List.map (fun port -> Vec3.add frame.Position (Rotation.apply frame.Rotation port.Position)) |> List.sortBy _.X
    let shared,free = ports.Head,List.last ports
    let pin id point =
        {Id=id;Part="4459.dat";Color=8;Position=Vec3.add point (Vec3.create 0 0 10)
         Rotation={X=Vec3.create 0 0 1;Y=Vec3.create 1 0 0;Z=Vec3.create 0 1 0}}
    let motor = {Goal="motor-bearing-support";Socket=shared;Plane=20.0;Bodies=["chassis"]
                 Alternatives=[[pin "motor-shared" shared];[pin "motor-free" free]]}
    let battery = {Goal="battery-support";Socket=shared;Plane=20.0;Bodies=["chassis"]
                   Alternatives=[[pin "battery-shared" shared]]}
    let problem = WalkerPackaging.supportProblem scope [frame] [] [motor;battery]
    let jointInitial = ConstructionSearch.create scope
                           [goal "scaffold" [];goal motor.Goal ["scaffold"];goal battery.Goal ["scaffold"]] 64 16
    let jointProof = ConstructionSearch.apply jointInitial.Frontier.Head (step "scaffold" "inherited" [])
    let clearKey = Export.sha256(Realization.json motor.Alternatives[1])
    let sharedKey = Export.sha256(Realization.json motor.Alternatives[0])
    let allocation =
        match solver.Plan(jointProof,[],problem) with
        | JointPlanned assignments when List.contains (motor.Goal,clearKey) assignments -> assignments
        | result -> failwithf "Expected joint allocation to reserve the shared socket for the battery, got %A" result
    let blockedPrefix = ConstructionSearch.apply jointProof (step motor.Goal sharedKey ["scaffold"])
    match solver.Plan(blockedPrefix,[],problem) with
    | JointBlocked(goals,bindings) when List.contains battery.Goal goals && List.contains (motor.Goal,sharedKey) bindings -> ()
    | result -> failwithf "Expected a shared-socket explanation for the fixed motor route, got %A" result
    let occupied = WalkerPackaging.supportProblem scope [frame] [pin "retained-pin" free] [motor;battery]
    match solver.Plan(jointProof,[],occupied) with
    | JointBlocked _ -> ()
    | result -> failwithf "A retained pin must make its socket unavailable, got %A" result
    let receiver = {battery with Goal="receiver-support";Alternatives=[[pin "receiver-shared" shared]]}
    let crowded = {WalkerPackaging.supportProblem scope [frame] [] [motor;battery;receiver] with Conflicts=[]}
    let crowdedProof = {jointProof with Goals=jointProof.Goals @ [goal receiver.Goal ["scaffold"]]}
    match solver.Plan(crowdedProof,[],crowded) with
    | JointBlocked(goals,_) when List.contains battery.Goal goals && List.contains receiver.Goal goals -> ()
    | result -> failwithf "Socket capacity alone must forbid three competing users, got %A" result
    if solver.Check(repaired,assumed)<>Consistent then failwith "Joint socket constraints leaked into ordinary proof checking."
    {|
        example="symbolic-trudge-support-v1"
        unsatCore=core |> List.map (fun (goal,choice) -> {|goal=goal;choice=choice|})
        untestedRoutePreserved=true;repairInvalidatedOldDomains=true;dynamicTrials=0
        jointAllocation=allocation |> List.map (fun (goal,choice) -> {|goal=goal;choice=choice|})
        sharedSocketReserved=true;retainedSocketRespected=true
        authority="Symbolic conflicts and catalog socket allocation; no complete build or physical qualification."
    |}
printfn "%s" (Realization.json report)
