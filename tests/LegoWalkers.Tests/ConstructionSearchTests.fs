namespace LegoWalkers

open System
open System.Text.Json
open Xunit

module ConstructionSearchTests =
    let private goal id requires external =
        {Id=id;Kind="test";Subject=id;Requires=requires;External=external}
    let private choice id value dependencies parts bodies introduces =
        {Step={Goal=id;Rule="test-production";Choice=value;Dependencies=dependencies
               Parts=parts;Bodies=bodies;Cost=0.0;Introduced=[];Bonds=[]};Introduces=introduces}
    let private seed goals = ConstructionSearch.create "frozen-request-oracle" goals 64 16
    let private verdict exact =
        {Scope="frozen-request-oracle";EvidenceSha256="frozen-evidence";Reason="rigid-body-partition"
         Parts=["axle-part"];Bodies=["output"];Exact=exact}
    let private expansion (proof: ConstructionProof) _ =
        let values = proof.Steps |> List.map (fun s -> s.Goal,s.Choice) |> Map.ofList
        if not(values.ContainsKey "frame") then
            Branch("frame",[choice "frame" "retained" [] ["frame-part"] ["chassis"] []])
        elif not(values.ContainsKey "axle") then
            Branch("axle",["long";"short"] |> List.map (fun value ->
                choice "axle" value ["frame"] ["axle-part"] ["output"] []))
        elif not(values.ContainsKey "brace") then
            Branch("brace",["near";"far"] |> List.map (fun value ->
                choice "brace" value ["frame"] ["brace-part"] ["chassis"] []))
        else Built(values["axle"]+"-"+values["brace"],proof)
    let private fixture () =
        seed [goal "frame" [] false;goal "axle" ["frame"] false
              goal "brace" ["frame"] false;goal "oracle" [] true]

    [<Fact>]
    let ``productions discharge open obligations and record prerequisites`` () =
        let proof = (seed [goal "a" [] false;goal "b" ["a"] false]).Frontier.Head
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.apply proof (choice "b" "x" [] [] [] []) |> ignore) |> ignore
        let first = ConstructionSearch.apply proof (choice "a" "x" [] ["part"] ["body"] [])
        Assert.Equal("b",(ConstructionSearch.openGoals first).Head.Id)
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.apply first (choice "b" "x" ["a"] ["part"] ["body"] []) |> ignore) |> ignore
        let finished = ConstructionSearch.apply first (choice "b" "x" ["a"] [] [] [])
        Assert.Empty(ConstructionSearch.openGoals finished)

    [<Fact>]
    let ``cycles and duplicate obligation IDs are rejected`` () =
        Assert.Throws<ArgumentException>(fun () ->
            seed [goal "a" ["b"] false;goal "b" ["a"] false] |> ignore) |> ignore
        Assert.Throws<ArgumentException>(fun () ->
            seed [goal "a" [] false;goal "a" [] false] |> ignore) |> ignore

    [<Fact>]
    let ``incomplete local construction cannot become a witness or an acceptance`` () =
        let state = seed [goal "local" [] false;goal "oracle" [] true]
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.next (fun _ _ -> Built("bad",())) state |> ignore) |> ignore
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.accept "hash" {state with Pending=Some state.Frontier.Head} |> ignore) |> ignore

    [<Fact>]
    let ``exact feedback backtracks through part provenance without changing the scaffold`` () =
        let state,first = ConstructionSearch.next expansion (fixture ())
        let rejected = ConstructionSearch.reject (verdict true) state
        Assert.Equal<(string * string) list>(first.Value.Steps |> List.map (fun s -> s.Goal,s.Choice),rejected.Nogoods.Head.Bindings)
        Assert.Equal<string list>(["frame";"axle"],rejected.Nogoods.Head.Targets)
        let resumed,second = ConstructionSearch.next expansion rejected
        Assert.Equal(first.Value.Steps.Head,second.Value.Steps.Head)
        Assert.Equal("short",second.Value.Steps[1].Choice)
        Assert.Equal(2,resumed.SeenWitnesses.Length)
        Assert.True(ConstructionSearch.openGoals second.Value |> List.forall _.External)

    [<Fact>]
    let ``direct oracle contributors precede cheaper incidental dependencies`` () =
        let initial = seed [goal "frame" [] false;goal "aux" ["frame"] false
                            goal "axle" ["frame"] false;goal "bearing" ["aux";"axle"] false;goal "oracle" [] true]
        let expand (proof: ConstructionProof) _ =
            let values = proof.Steps |> List.map (fun s -> s.Goal,s.Choice) |> Map.ofList
            if not(values.ContainsKey "frame") then Branch("frame",[choice "frame" "same" [] [] ["chassis"] []])
            elif not(values.ContainsKey "aux") then
                Branch("aux",["a";"b"] |> List.map (fun v -> choice "aux" v ["frame"] ["aux-part"] ["other-shaft"] []))
            elif not(values.ContainsKey "axle") then
                Branch("axle",["a";"b"] |> List.map (fun v -> choice "axle" v ["frame"] ["axle-part"] ["output"] []))
            elif not(values.ContainsKey "bearing") then
                Branch("bearing",[choice "bearing" "same" ["aux";"axle"] ["bearing-part"] ["output"] []])
            else Built(values["aux"]+"-"+values["axle"],proof)
        let pending,original = ConstructionSearch.next expand initial
        let rejected = ConstructionSearch.reject (verdict true) pending
        Assert.Contains("aux",rejected.Nogoods.Head.Targets)
        Assert.DoesNotContain("aux",rejected.Nogoods.Head.Focus)
        let _,repaired = ConstructionSearch.next expand rejected
        Assert.Equal(original.Value.Steps[1],repaired.Value.Steps[1])
        Assert.NotEqual<string>(original.Value.Steps[2].Choice,repaired.Value.Steps[2].Choice)

    [<Fact>]
    let ``advisory hints are soft and feedback cannot cross oracle scopes`` () =
        let state,_ = ConstructionSearch.next expansion (fixture ())
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.reject {(verdict true) with Scope="changed-library"} state |> ignore) |> ignore
        let soft = ConstructionSearch.reject (verdict false) state
        Assert.False(soft.Nogoods.Head.Feedback.Exact)
        let _,other = ConstructionSearch.next expansion soft
        Assert.True(other.IsSome)

    [<Fact>]
    let ``finite conflict reduction checks jointly freed choices and respects its budget`` () =
        let bindings = ["layout","blocked";"a","0";"b","0"]
        let domains = ["a",["0";"1"];"b",["0";"1"]]
        let mutable checkedAssignments = []
        let rejects assignment =
            checkedAssignments <- assignment::checkedAssignments
            let values = Map.ofList assignment
            Some(not(values["a"]="1" && values["b"]="1"))
        let reduced,checks = ConstructionSearch.minimizeFiniteConflict 8 bindings domains rejects
        Assert.Equal<(string * string) list>(["layout","blocked";"b","0"],reduced)
        Assert.Equal(6,checks)
        Assert.Contains(checkedAssignments,fun values -> Map.ofList values |> fun map -> map["a"]="1" && map["b"]="1")
        let bounded,count = ConstructionSearch.minimizeFiniteConflict 1 bindings domains (fun _ -> failwith "Budget exceeded")
        Assert.Equal<(string * string) list>(bindings,bounded)
        Assert.Equal(0,count)
        let mutable calls = 0
        let incomplete,_ = ConstructionSearch.minimizeFiniteConflict 8 bindings domains (fun _ ->
            calls<-calls+1
            if calls=1 then Some true else None)
        Assert.Equal<(string * string) list>(bindings,incomplete)
        Assert.Equal(2,calls)

    [<Fact>]
    let ``covered conflicts generalize only jointly exhausted finite domains`` () =
        let bindings = ["layout","blocked";"a","0";"b","0"]
        let domains = ["a",["0";"1"];"b",["0";"1"]]
        let clauses =
            [for a,b in ["0","0";"0","1";"1","0"] do
                 yield ["layout","blocked";"a",a;"b",b]]
        let partial,_ = ConstructionSearch.minimizeCoveredConflict 32 bindings domains clauses
        Assert.Equal<(string * string) list>(["layout","blocked";"b","0"],partial)
        let complete,_ = ConstructionSearch.minimizeCoveredConflict 32 bindings domains
                            (["layout","blocked";"a","1";"b","1"]::clauses)
        Assert.Equal<(string * string) list>(["layout","blocked"],complete)
        let unrelated = clauses |> List.map (fun clause ->
            clause |> List.map (fun (goal,value) -> goal,if goal="layout" then "other" else value))
        let unchanged,_ = ConstructionSearch.minimizeCoveredConflict 32 bindings domains unrelated
        Assert.Equal<(string * string) list>(bindings,unchanged)

    [<Fact>]
    let ``family slices reach other layouts and preserve deferred branches across checkpoints`` () =
        let goals = [{goal "layout" [] false with Kind="layout"}
                     goal "axle" ["layout"] false;goal "oracle" [] true]
        let initial = ConstructionSearch.create "fair-scope" goals 4 1
                      |> ConstructionSearch.withFamilyScheduling ["layout"] 4
        let mutable layouts = []
        let expand (proof: ConstructionProof) _ =
            let values = proof.Steps |> List.map (fun step -> step.Goal,step.Choice) |> Map.ofList
            if not(values.ContainsKey "layout") then
                Branch("layout",["a";"b";"c"] |> List.map (fun value -> choice "layout" value [] [] [] []))
            elif not(values.ContainsKey "axle") then
                layouts <- layouts @ [values["layout"]]
                let count = if values["layout"]="c" then 1 else 40
                Branch("axle",[for index in 1..count do yield choice "axle" (string index) ["layout"] [] [] []])
            elif values["layout"]="c" then Built("clear",proof)
            else Blocked {Gate="support";Outcome="reject";Code="blocked";Diagnostics=[]}
        let limited,witness = ConstructionSearch.next expand initial
        Assert.True(witness.IsNone)
        Assert.Equal(0,limited.Discarded)
        Assert.InRange(limited.Frontier.Length,0,1)
        Assert.NotEmpty(limited.Deferred)
        let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase)
        let checkpoint = JsonSerializer.Deserialize<ConstructionSearchState>(Realization.json limited,options)
        Assert.Equal(limited,checkpoint)
        let resumed,built = ConstructionSearch.next expand {checkpoint with MaximumStates=64}
        Assert.True(built.IsSome)
        Assert.Equal<string list>(["a";"b";"c"],layouts)
        Assert.InRange(resumed.Visited,1,20)
        Assert.NotEmpty(resumed.Deferred)

    [<Fact>]
    let ``active frontier window never drops a late viable production`` () =
        let initial = ConstructionSearch.create "deferred-scope" [goal "a" [] false;goal "oracle" [] true] 64 2
        let expand (proof: ConstructionProof) _ =
            if proof.Steps.IsEmpty then
                Branch("a",[for index in 0..19 do
                                yield {choice "a" (string index) [] [] [] [] with
                                           Step={ (choice "a" (string index) [] [] [] []).Step with Cost=float index}}])
            elif proof.Steps.Head.Choice="19" then Built("late",proof)
            else Blocked {Gate="support";Outcome="reject";Code="blocked";Diagnostics=[]}
        let state,witness = ConstructionSearch.next expand initial
        Assert.True(witness.IsSome)
        Assert.Equal("19",witness.Value.Steps.Head.Choice)
        Assert.Equal(21,state.Visited)
        Assert.Equal(0,state.Discarded)

    [<Fact>]
    let ``inconclusive constraint checks remain explicit and do not reject a viable proof`` () =
        let initial = seed [goal "frame" [] false;goal "oracle" [] true]
        let check _ _ = Inconclusive "symbolic-check-budget-exhausted"
        let expand (proof: ConstructionProof) _ =
            if proof.Steps.IsEmpty then Branch("frame",[choice "frame" "same" [] [] [] []])
            else Built("witness",proof)
        let state,witness = ConstructionSearch.nextObservedWithConstraints (Some check) (fun _ _ _ -> ()) expand initial
        Assert.True(witness.IsSome)
        Assert.Equal(0,state.ConstraintPruned)
        Assert.Equal<string list>(["symbolic-check-budget-exhausted"],state.ConstraintWarnings)
        Assert.Equal(2,state.ConstraintChecks)
        Assert.Equal("frame",state.Domains.Head.Goal)

    [<Fact>]
    let ``constraint explanations prune before expansion and must match proof assumptions`` () =
        let initial = seed [goal "frame" [] false;goal "oracle" [] true]
        let mutable expanded = []
        let expand (proof: ConstructionProof) _ =
            if proof.Steps.IsEmpty then
                Branch("frame",["blocked";"clear"] |> List.map (fun value -> choice "frame" value [] [] [] []))
            else
                expanded <- expanded @ [proof.Steps.Head.Choice]
                Built("witness",proof)
        let check _ (proof: ConstructionProof) =
            if proof.Steps |> List.exists (fun step -> step.Choice="blocked") then Inconsistent ["frame","blocked"]
            else Consistent
        let state,witness = ConstructionSearch.nextObservedWithConstraints (Some check) (fun _ _ _ -> ()) expand initial
        Assert.True(witness.IsSome)
        Assert.Equal<string list>(["clear"],expanded)
        Assert.Equal(1,state.ConstraintPruned)
        let invalidCheck _ _ = Inconsistent ["frame","not-an-assumption"]
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.nextObservedWithConstraints (Some invalidCheck) (fun _ _ _ -> ()) expand initial |> ignore) |> ignore

    [<Fact>]
    let ``joint guidance selects a preferred branch without changing immutable production cost`` () =
        let initial = seed [goal "frame" [] false;goal "oracle" [] true]
        let cheap = choice "frame" "cheap" [] [] [] []
        let planned = {choice "frame" "planned" [] [] [] [] with
                           Step={ (choice "frame" "planned" [] [] [] []).Step with Cost=10.0}}
        let expand (proof: ConstructionProof) _ =
            if proof.Steps.IsEmpty then GuidedBranch("frame",[cheap;planned],"planned")
            elif proof.Steps.Head=planned.Step then Built("planned",proof)
            else Blocked {Gate="support";Outcome="reject";Code="unplanned";Diagnostics=[]}
        let state,witness = ConstructionSearch.next expand initial
        Assert.True(witness.IsSome)
        Assert.Equal(2,state.Visited)
        Assert.Equal(planned.Step,witness.Value.Steps.Head)
        Assert.Equal<string list>(["cheap";"planned"],state.Domains.Head.Choices)

    [<Fact>]
    let ``local causal conflict skips unrelated siblings and survives checkpoint continuation`` () =
        let initial = seed [goal "layout" [] false;goal "easy" ["layout"] false
                            goal "socket" ["layout"] false;goal "oracle" [] true]
        let mutable conflicts = 0
        let expand (proof: ConstructionProof) _ =
            let values = proof.Steps |> List.map (fun s -> s.Goal,s.Choice) |> Map.ofList
            if not(values.ContainsKey "layout") then
                Branch("layout",["blocked";"clear"] |> List.map (fun v -> choice "layout" v [] [] [] []))
            elif not(values.ContainsKey "easy") then
                Branch("easy",["a";"b";"c";"d"] |> List.map (fun v -> choice "easy" v ["layout"] [] [] []))
            elif values["layout"]="blocked" then
                conflicts<-conflicts+1
                BlockedBy({Gate="support";Outcome="reject";Code="bounded-domain-empty";Diagnostics=["socket"]},
                          ["layout","blocked"])
            elif not(values.ContainsKey "socket") then
                Branch("socket",[choice "socket" "route" ["layout"] [] [] []])
            else Built("witness",proof)
        let state,witness = ConstructionSearch.next expand initial
        Assert.True(witness.IsSome)
        Assert.Equal(1,conflicts)
        Assert.Equal("clear",witness.Value.Steps.Head.Choice)
        Assert.Equal(3,state.Pruned)
        Assert.Equal<(string * string) list>(["layout","blocked"],state.Nogoods.Head.Bindings)
        let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase)
        let checkpoint = JsonSerializer.Deserialize<ConstructionSearchState>(Realization.json state,options)
        Assert.Equal<ConstructionNogood list>(state.Nogoods,checkpoint.Nogoods)
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.next (fun _ _ ->
                BlockedBy({Gate="support";Outcome="reject";Code="bad";Diagnostics=[]},["layout","unknown"])) initial
            |> ignore) |> ignore

    [<Fact>]
    let ``checkpoint roundtrip reproduces continuation and bound accounting`` () =
        let pending,_ = ConstructionSearch.next expansion (fixture ())
        let state = ConstructionSearch.reject (verdict true) pending
        let options = JsonSerializerOptions(PropertyNamingPolicy=JsonNamingPolicy.CamelCase)
        let restored = JsonSerializer.Deserialize<ConstructionSearchState>(Realization.json state,options)
        let expected,witness = ConstructionSearch.next expansion state
        let actual,replayed = ConstructionSearch.next expansion restored
        Assert.Equal(expected,actual)
        Assert.Equal(witness,replayed)
        let bound = ConstructionSearch.create "scope" [goal "a" [] false] 1 1
        let limited,_ = ConstructionSearch.next (fun _ _ ->
            Branch("a",[choice "a" "one" [] [] [] [];choice "a" "two" [] [] [] []])) bound
        Assert.Equal("state-bound-reached",limited.Status)
        Assert.Equal(1,limited.Visited)
        Assert.Equal(0,limited.Discarded)
        Assert.Single(limited.Deferred) |> ignore

    [<Fact>]
    let ``observing search exposes live partial terms without changing decisions or accounting`` () =
        let events = ResizeArray<_>()
        let observe (state: ConstructionSearchState) proof (outcome: ConstructionExpansion<ConstructionProof> option) =
            events.Add(state.Visited,proof,outcome.IsSome)
        let expected,witness = ConstructionSearch.next expansion (fixture ())
        let actual,observed = ConstructionSearch.nextObserved observe expansion (fixture ())
        Assert.Equal(expected,actual)
        Assert.Equal(witness,observed)
        Assert.True(events |> Seq.exists (fun (_,proof,finished) -> not finished && not((ConstructionSearch.openGoals proof).IsEmpty)))
        Assert.Equal(expected.Visited*2,events.Count)

    [<Fact>]
    let ``oracle acceptance closes external goals only after local witnesses`` () =
        let pending,proof = ConstructionSearch.next expansion (fixture ())
        Assert.Equal("oracle",(ConstructionSearch.openGoals proof.Value).Head.Id)
        let state,accepted = ConstructionSearch.accept "immutable-oracle-report" pending
        Assert.Equal("oracle-accepted",state.Status)
        Assert.Empty(ConstructionSearch.openGoals accepted)
        Assert.Equal("external-oracle",accepted.Steps |> List.last |> _.Rule)

    [<Fact>]
    let ``explicit external connectivity cannot be certified by a generic acceptance`` () =
        let local = goal "frame" [] false
        let connectivity = {goal "library-connectivity" ["frame"] true with Kind="exact-rigid-connectivity"}
        let proof = (seed [local;connectivity;goal "oracle" ["library-connectivity"] true]).Frontier.Head
                    |> fun proof -> ConstructionSearch.apply proof (choice "frame" "fixed" [] ["beam"] ["chassis"] [])
        let state = {seed proof.Goals with Pending=Some proof}
        Assert.Throws<ArgumentException>(fun () -> ConstructionSearch.accept "report" state |> ignore) |> ignore
        Assert.Throws<ArgumentException>(fun () -> ConstructionSearch.acceptVerified "report" ["oracle"] state |> ignore) |> ignore
        let _,accepted = ConstructionSearch.acceptVerified "report" ["oracle";"library-connectivity"] state
        Assert.Empty(ConstructionSearch.openGoals accepted)
        Assert.Equal("library-connectivity",accepted.Steps[1].Goal)

    [<Theory>]
    [<InlineData("rigid-bond-engagement")>]
    [<InlineData("joint-port-engagement")>]
    let ``precise engagement feedback does not focus every chassis production`` reason =
        let state,_ = ConstructionSearch.next expansion (fixture ())
        let rejected = ConstructionSearch.reject {(verdict true) with Reason=reason;Parts=["brace-part"];Bodies=["chassis"]} state
        Assert.Equal<string list>(["brace"],rejected.Nogoods.Head.Focus)

    [<Fact>]
    let ``partition feedback retains affected shaft productions even when displaced members have other producers`` () =
        let state,_ = ConstructionSearch.next expansion (fixture ())
        let rejected = ConstructionSearch.reject {(verdict true) with Parts=["brace-part"];Bodies=["output"]} state
        Assert.Contains("brace",rejected.Nogoods.Head.Focus)
        Assert.Contains("axle",rejected.Nogoods.Head.Focus)

    [<Fact>]
    let ``joint preservation alone requires explicit oracle certification`` () =
        let frame = goal "frame" [] false
        let preservation = {goal "preserve-pivot" ["frame"] true with Kind="exact-joint-preservation"}
        let proof = (seed [frame;preservation;goal "oracle" [] true]).Frontier.Head
                    |> fun proof -> ConstructionSearch.apply proof (choice "frame" "fixed" [] ["beam"] ["chassis"] [])
        let state = {seed proof.Goals with Pending=Some proof}
        Assert.Throws<ArgumentException>(fun () -> ConstructionSearch.accept "report" state |> ignore) |> ignore
        Assert.Throws<ArgumentException>(fun () -> ConstructionSearch.acceptVerified "report" ["oracle"] state |> ignore) |> ignore
        let _,accepted = ConstructionSearch.acceptVerified "report" ["oracle";"preserve-pivot"] state
        Assert.Empty(ConstructionSearch.openGoals accepted)

    [<Fact>]
    let ``rigid bonds must record their physical producer dependencies`` () =
        let proof = (seed [goal "parts" [] false;goal "bond" ["parts"] false]).Frontier.Head
        let proof = ConstructionSearch.apply proof (choice "parts" "fixed" [] ["a";"b";"pin"] ["chassis"] [])
        let bond = {Connector="pin";PartA="a";PositionA=Vec3.zero;AxisA=Vec3.create 0 0 1
                    PartB="b";PositionB=Vec3.create 0 0 20;AxisB=Vec3.create 0 0 1}
        let selected = choice "bond" "fixed" ["parts"] [] ["chassis"] []
        let selected = {selected with Step={selected.Step with Bonds=[bond]}}
        ConstructionSearch.apply proof selected |> ignore
        Assert.Throws<ArgumentException>(fun () ->
            ConstructionSearch.apply proof {selected with Step={selected.Step with Dependencies=[]}} |> ignore) |> ignore

    [<Fact>]
    let ``dependency refresh retains only previously introduced obligations`` () =
        let original = seed [goal "root" [] false;goal "oracle" [] true]
        let expand (proof: ConstructionProof) _ =
            if proof.Steps.IsEmpty then
                Branch("root",["a";"b"] |> List.map (fun value ->
                    choice "root" value [] ["axle-part"] ["output"] [goal "child" ["root"] false]))
            elif proof.Steps.Length=1 then
                Branch("child",[choice "child" "fixed" ["root"] [] [] []])
            else Built(proof.Steps.Head.Choice,())
        let pending,_ = ConstructionSearch.next expand original
        let resumed,witness = pending |> ConstructionSearch.reject (verdict true) |> ConstructionSearch.next expand
        Assert.True(witness.IsSome)
        Assert.Equal(3,resumed.Pending.Value.Goals.Length)
