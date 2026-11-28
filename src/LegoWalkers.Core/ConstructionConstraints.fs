namespace LegoWalkers

open System
open System.Collections.Generic
open System.Diagnostics
open System.IO
open System.Text.RegularExpressions

type ConstructionJointSocket = {
    Id: string
    Part: string
    Position: Vec3
    Axis: Vec3
    OccupiedBy: string list
}

type ConstructionJointChoice = {
    Key: string
    Parts: string list
    Sockets: string list
}

type ConstructionJointRequirement = {
    Goal: string
    Socket: Vec3
    Plane: float
    Bodies: string list
    Choices: ConstructionJointChoice list
}

type ConstructionJointConflict = {
    Left: string * string
    Right: string * string
    Reason: string
}

type ConstructionJointProblem = {
    Id: string
    Scope: string
    Sockets: ConstructionJointSocket list
    Requirements: ConstructionJointRequirement list
    Conflicts: ConstructionJointConflict list
    Rejections: (string * string * GateDecision) list
}

type ConstructionJointPlan =
    | JointPlanned of assignments: (string * string) list
    | JointBlocked of goals: string list * bindings: (string * string) list
    | JointInconclusive of reason: string

type ConstructionJointPlanner = {
    Solve: ConstructionProof -> ConstructionNogood list -> ConstructionJointProblem -> ConstructionJointPlan
    Report: ConstructionJointProblem -> ConstructionJointPlan -> unit
}

type Z3ConstructionSolver(executable: string, scope: string) =
    let path = Path.GetFullPath executable
    let start =
        if not(File.Exists path) then invalidArg "constraint-solver" $"Solver executable does not exist: {path}"
        let info = ProcessStartInfo(path)
        info.ArgumentList.Add "-in"
        info.ArgumentList.Add "-smt2"
        info.UseShellExecute <- false
        info.RedirectStandardInput <- true
        info.RedirectStandardOutput <- true
        info.RedirectStandardError <- true
        info
    let solverProcess = Process.Start start
    let errors = solverProcess.StandardError.ReadToEndAsync()
    let variables = Dictionary<string,ResizeArray<string>>()
    let mutable registeredDomains: ConstructionDomain list = []
    let mutable registeredClauses: (string * string) list list = []
    let mutable queryNumber = 0
    let mutable version = ""
    let write (text: string) = solverProcess.StandardInput.WriteLine text
    let query command =
        queryNumber <- queryNumber+1
        let marker = $"constraint-response-{queryNumber}"
        write command
        write $"(echo \"{marker}\")"
        solverProcess.StandardInput.Flush()
        let lines = ResizeArray<string>()
        let mutable finished = false
        while not finished do
            let line = solverProcess.StandardOutput.ReadLineAsync().WaitAsync(TimeSpan.FromSeconds 5.0).GetAwaiter().GetResult()
            if isNull line then
                let detail = if solverProcess.HasExited then errors.GetAwaiter().GetResult() else "Solver closed its output."
                invalidOp $"Constraint solver terminated unexpectedly: {detail}"
            elif line.StartsWith("(error",StringComparison.Ordinal) then
                invalidOp $"Constraint solver rejected the model: {line}"
            elif line=marker then finished <- true
            else lines.Add line
        String.concat "\n" lines
    let initialize () =
        write "(set-option :print-success false)"
        write "(set-option :produce-unsat-cores true)"
        write "(set-option :timeout 2000)"
        write "(set-logic QF_UF)"
    let reset () =
        write "(reset)"
        variables.Clear()
        registeredDomains <- []
        registeredClauses <- []
        initialize ()
    let symbol binding = "choice_"+Export.sha256(Realization.json binding)
    let declare ((goal,_) as binding) =
        let name = symbol binding
        let values =
            match variables.TryGetValue goal with
            | true,values -> values
            | _ ->
                let values = ResizeArray<string>()
                variables.Add(goal,values)
                values
        if not(values.Contains name) then
            write $"(declare-const {name} Bool)"
            for other in values do write $"(assert (not (and {name} {other})))"
            values.Add name
        name
    let conjunction bindings =
        match bindings |> List.map declare with
        | [] -> "true"
        | names -> "(and "+String.concat " " names+")"
    let readCore known =
        let response = query "(get-unsat-core)"
        let tokens = response.Replace("("," ").Replace(")"," ").Split([|' ';'\n';'\r';'\t'|],StringSplitOptions.RemoveEmptyEntries) |> Set.ofArray
        if not(Set.isSubset tokens known) then invalidOp "Constraint solver returned an unknown core assumption."
        tokens
    let stop () =
        try
            if not solverProcess.HasExited then
                write "(exit)"
                solverProcess.StandardInput.Flush()
                if not(solverProcess.WaitForExit 3000) then solverProcess.Kill true
        finally
            if not solverProcess.HasExited then solverProcess.Kill true
            solverProcess.WaitForExit()
            solverProcess.Dispose()
    do
        let mutable ready = false
        try
            initialize ()
            if query "(get-info :name)"<>"(:name \"Z3\")" then
                invalidArg "constraint-solver" "The finite constraint backend requires Z3."
            version <- query "(get-info :version)"
            ready <- true
        finally
            if not ready then stop ()

    member _.Description = {|
        backend="z3-finite-construction-v2";executable=path;version=version
        binarySha256=Convert.ToHexString(Security.Cryptography.SHA256.HashData(File.ReadAllBytes path)).ToLowerInvariant()
        authority="Finite domains, scaffold sockets and F#-explained route conflicts; concrete replay and walking qualification remain separate."
    |}

    member _.Check(state: ConstructionSearchState, proof: ConstructionProof) =
        if state.Scope<>scope || proof.Scope<>scope then
            invalidArg "constraint-solver" "Solver facts cannot cross construction request/oracle scopes."
        let exact = state.Nogoods |> List.filter (fun nogood -> nogood.Feedback.Exact && nogood.Feedback.Scope=scope)
                    |> List.map _.Bindings
        let extends previous current =
            List.length previous<=List.length current && List.take (List.length previous) current=previous
        if not(extends registeredDomains state.Domains) || not(extends registeredClauses exact) then
            reset ()
        for domain in state.Domains |> List.skip registeredDomains.Length do
            if String.IsNullOrWhiteSpace domain.Goal || List.contains domain.Goal (List.map fst domain.Bindings)
               || (domain.Bindings |> List.map fst |> Set.ofList).Count<>domain.Bindings.Length
               || (Set.ofList domain.Choices).Count<>domain.Choices.Length
               || (domain.Bindings |> List.exists (fun (goal,choice) -> String.IsNullOrWhiteSpace goal || String.IsNullOrWhiteSpace choice))
               || domain.Choices |> List.exists String.IsNullOrWhiteSpace then
                invalidArg "constraint-domain" "A conditional finite domain needs an open goal and distinct bindings/choices."
            let condition = conjunction domain.Bindings
            let choices = domain.Choices |> List.map (fun choice -> declare(domain.Goal,choice))
            let alternatives = if choices.IsEmpty then "false" else "(or "+String.concat " " choices+")"
            write $"(assert (=> {condition} {alternatives}))"
        for bindings in exact |> List.skip registeredClauses.Length do
            write ("(assert (not "+conjunction bindings+"))")
        registeredDomains <- state.Domains
        registeredClauses <- exact
        let assumptions = proof.Steps |> List.map (fun step -> step.Goal,step.Choice)
        let names = assumptions |> List.map declare |> String.concat " "
        match query $"(check-sat-assuming ({names}))" with
        | "sat" -> Consistent
        | "unsat" ->
            let known = assumptions |> List.map symbol |> Set.ofList
            let tokens = readCore known
            Inconsistent(assumptions |> List.filter (symbol >> tokens.Contains))
        | "unknown" -> Inconclusive(query "(get-info :reason-unknown)")
        | response -> invalidOp $"Unexpected constraint solver response: {response}"

    member _.Plan(proof: ConstructionProof, nogoods: ConstructionNogood list, problem: ConstructionJointProblem) =
        ConstructionSearch.validate proof |> ignore
        if proof.Scope<>scope || problem.Scope<>scope then
            invalidArg "joint-support-plan" "Joint support facts cannot cross construction scopes."
        let goals = problem.Requirements |> List.map _.Goal |> Set.ofList
        let sockets = problem.Sockets |> List.map (fun socket -> socket.Id,socket) |> Map.ofList
        let available =
            problem.Requirements |> List.collect (fun requirement ->
                requirement.Choices |> List.map (fun choice -> requirement.Goal,choice.Key)) |> Set.ofList
        if goals.Count<>problem.Requirements.Length || sockets.Count<>problem.Sockets.Length
           || available.Count<>(problem.Requirements |> List.sumBy (fun requirement -> requirement.Choices.Length))
           || (problem.Requirements |> List.exists (fun requirement -> String.IsNullOrWhiteSpace requirement.Goal))
           || (problem.Requirements |> List.exists (fun requirement -> requirement.Choices |> List.exists (fun choice ->
                String.IsNullOrWhiteSpace choice.Key || choice.Sockets |> List.exists (sockets.ContainsKey >> not))))
           || (problem.Conflicts |> List.exists (fun conflict ->
                not(available.Contains conflict.Left && available.Contains conflict.Right)))
           || (problem.Rejections |> List.exists (fun (goal,choice,gate) ->
                gate.Outcome<>"reject" || not(available.Contains(goal,choice)))) then
            invalidArg "joint-support-plan" "Joint requirements need unique goals, choices, sockets and valid incompatibility references."
        let selected = proof.Steps |> List.choose (fun step -> if goals.Contains step.Goal then Some(step.Goal,step.Choice) else None)
        let outside = selected |> List.filter (available.Contains >> not)
        if not outside.IsEmpty then
            JointInconclusive("Selected routes outside the initial bounded joint pool: "+String.concat ", " (outside |> List.map fst))
        else
            reset ()
            try
                let fixedBindings = proof.Steps |> List.filter (fun step -> not(goals.Contains step.Goal))
                                    |> List.map (fun step -> step.Goal,step.Choice)
                let fixedValues = Map.ofList fixedBindings
                let needed = problem.Requirements |> List.map (fun requirement ->
                    let name = "need_"+Export.sha256 requirement.Goal
                    write $"(declare-const {name} Bool)"
                    let names = requirement.Choices |> List.map (fun choice -> declare(requirement.Goal,choice.Key))
                    let alternatives = if names.IsEmpty then "false" else "(or "+String.concat " " names+")"
                    write $"(assert (=> {name} {alternatives}))"
                    requirement.Goal,name)
                let socketUsers = Dictionary<string,ResizeArray<string>>()
                for requirement in problem.Requirements do
                    for choice in requirement.Choices do
                        let name = declare(requirement.Goal,choice.Key)
                        for socketId in List.distinct choice.Sockets do
                            if sockets[socketId].OccupiedBy |> List.exists (fun id -> not(List.contains id choice.Parts)) then
                                write $"(assert (not {name}))"
                            let users =
                                match socketUsers.TryGetValue socketId with
                                | true,users -> users
                                | _ ->
                                    let users = ResizeArray<string>()
                                    socketUsers.Add(socketId,users)
                                    users
                            users.Add name
                // Linear at-most-one encoding using portable Boolean SMT-LIB.
                for KeyValue(socketId,users) in socketUsers do
                    if users.Count>1 then
                        let mutable previous = users[0]
                        for index in 1..users.Count-1 do
                            let user = users[index]
                            write $"(assert (not (and {previous} {user})))"
                            if index<users.Count-1 then
                                let next = "occupied_"+Export.sha256 socketId+"_"+string index
                                write $"(declare-const {next} Bool)"
                                write $"(assert (= {next} (or {previous} {user})))"
                                previous <- next
                for conflict in problem.Conflicts do
                    write ("(assert (not "+conjunction [conflict.Left;conflict.Right]+"))")
                for goal,choice,_ in problem.Rejections do
                    if not(available.Contains(goal,choice)) then invalidArg "joint-support-plan" "A rejected route is not in the joint domain."
                    write ("(assert (not "+declare(goal,choice)+"))")
                for nogood in nogoods do
                    if nogood.Feedback.Exact && nogood.Feedback.Scope=scope
                       && (nogood.Bindings |> List.forall (fun ((goal,choice) as binding) ->
                            if goals.Contains goal then available.Contains binding else Map.tryFind goal fixedValues=Some choice)) then
                        write ("(assert (not "+conjunction nogood.Bindings+"))")
                let assumptions = (needed |> List.map snd) @ (fixedBindings @ selected |> List.map declare)
                let check extra =
                    let names = assumptions @ (extra |> List.map declare) |> String.concat " "
                    query $"(check-sat-assuming ({names}))"
                match check [] with
                | "unsat" ->
                    let core = readCore (Set.ofList assumptions)
                    JointBlocked(needed |> List.choose (fun (goal,name) -> if core.Contains name then Some goal else None),
                                 (fixedBindings @ selected) |> List.filter (symbol >> core.Contains))
                | "unknown" -> JointInconclusive(query "(get-info :reason-unknown)")
                | "sat" ->
                    let mutable preferred = []
                    let mutable unknown = None
                    for requirement in problem.Requirements do
                        if unknown.IsNone && not(selected |> List.exists (fun (goal,_) -> goal=requirement.Goal)) then
                            let mutable found = false
                            for choice in requirement.Choices do
                                if not found && unknown.IsNone then
                                    let binding = requirement.Goal,choice.Key
                                    match check (preferred @ [binding]) with
                                    | "sat" -> preferred <- preferred @ [binding];found <- true
                                    | "unsat" -> ()
                                    | "unknown" -> unknown <- Some(query "(get-info :reason-unknown)")
                                    | response -> invalidOp $"Unexpected joint planner response: {response}"
                            if not found && unknown.IsNone then invalidOp "A satisfiable joint support plan lost every candidate."
                    match unknown with
                    | Some reason -> JointInconclusive reason
                    | None ->
                        // An unsat preference probe invalidates the previous model.
                        match check preferred with
                        | "unknown" -> JointInconclusive(query "(get-info :reason-unknown)")
                        | "sat" ->
                            let names = available |> Set.toList |> List.map declare |> String.concat " "
                            let response = query $"(get-value ({names}))"
                            let values =
                                Regex.Matches(response,@"\((choice_[0-9a-f]+)\s+(true|false)\)")
                                |> Seq.cast<Match> |> Seq.map (fun entry -> entry.Groups[1].Value,entry.Groups[2].Value="true")
                                |> Map.ofSeq
                            if values.Count<>available.Count then invalidOp "Joint planner returned an incomplete route assignment."
                            let assignments = available |> Set.toList |> List.filter (fun binding -> values[symbol binding])
                            if assignments.Length<>goals.Count || (assignments |> List.map fst |> Set.ofList).Count<>goals.Count then
                                invalidOp "Joint planner must select exactly one route per requirement."
                            JointPlanned assignments
                        | response -> invalidOp $"Unexpected final joint planner response: {response}"
                | response -> invalidOp $"Unexpected joint planner response: {response}"
            finally
                reset ()

    interface IDisposable with
        member _.Dispose() = stop ()
