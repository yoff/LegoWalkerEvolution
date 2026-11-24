namespace LegoWalkers

open System

module WalkerInnerFeet =
    let version = "parallel-ankle-soles-v4"
    let private reject detail =
        invalidArg "inner-foot" detail

    let private addSelected (s: ScreenedMotion) (g: WalkerGenome) selected =
        try
            let center = g.Feet |> List.averageBy _.Depth
            let additions = [
                for foot in s.Program.Feet do
                  if Set.contains foot.Id selected then
                    let choice = g.Feet |> List.find (fun c -> c.Foot=foot.Id)
                    if not (defaultArg choice.AdditionalContacts []).IsEmpty then
                        reject $"Foot {foot.Id} already has an additional contact surface."
                    let body = g.Bodies |> List.find (fun b -> b.Body=foot.Port.Body)
                    let sole = body.Parts |> List.find (fun p -> p.Id=choice.Part)
                    if sole.Part<>"3701.dat" || abs sole.Rotation.Z.Z<0.999999 then
                        reject $"Foot {foot.Id} requires a transverse 3701 sole."
                    let hinges = s.Program.Hinges |> List.filter (fun h -> h.A.Body=body.Body || h.B.Body=body.Body)
                    let pivots = g.Connections |> List.filter (fun c ->
                        c.Hinges |> List.exists (fun id -> hinges |> List.exists (fun h -> h.Id=id)))
                    if hinges.Length<>2 || pivots.Length<>2
                       || (pivots |> List.exists (fun p ->
                           p.Hinges.Length<>1 || (Catalog.get p.Placement.Part).Connector<>Some FrictionlessPin)) then
                        reject $"Foot {foot.Id} requires two independent smooth-pin ankle joints."
                    let linkDepths =
                        g.Bodies |> List.filter (fun b -> b.Body<>body.Body) |> List.collect _.Parts
                        |> List.filter (fun p -> pivots |> List.exists (fun pivot -> WalkerStructural.engaged pivot.Placement p))
                        |> List.map _.Position.Z |> List.distinct
                    let layer =
                        match linkDepths with
                        | [depth] when abs(abs(depth-sole.Position.Z)-20.0)<1e-6 -> depth
                        | _ -> reject $"Foot {foot.Id} needs both leg links in the adjacent 20-LDU layer."
                    let depth = 2.0*layer-sole.Position.Z
                    let suffix = if abs(depth-center)<abs(sole.Position.Z-center) then "-inner" else "-outer"
                    let inner = {sole with Id=sole.Id+suffix;Position={sole.Position with Z=depth}}
                    let pins = pivots |> List.map (fun p ->
                        {p with Placement={p.Placement with Part="32556.dat";Position={p.Placement.Position with Z=layer}}})
                    yield body.Body,inner,pins,{choice with AdditionalContacts=Some [{Part=inner.Id;Depth=depth}]}
            ]
            let pins = additions |> List.collect (fun (_,_,pins,_) -> pins) |> List.map (fun p -> p.Group,p) |> Map.ofList
            let candidate = {
                g with
                    Bodies=g.Bodies |> List.map (fun body ->
                        match additions |> List.tryFind (fun (id,_,_,_) -> id=body.Body) with
                        | Some(_,inner,_,_) -> {body with Parts=body.Parts @ [inner]}
                        | None -> body)
                    Connections=g.Connections |> List.map (fun p -> Map.tryFind p.Group pins |> Option.defaultValue p)
                    Feet=g.Feet |> List.map (fun choice ->
                        additions |> List.tryFind (fun (_,_,_,updated) -> updated.Foot=choice.Foot)
                        |> Option.map (fun (_,_,_,updated) -> updated) |> Option.defaultValue choice)
            }
            if pins |> Map.exists (fun _ pin -> not(WalkerConstruction.pivotSupported s.Program candidate pin)) then
                reject "The 3L smooth pins do not cover every socket wall in the repaired ankle stack."
            match WalkerRealization.compile s candidate with
            | Error gate -> Error gate
            | Ok realized -> Ok realized.Genome
        with :? ArgumentException as error ->
            Error {Gate="lego-realizability";Outcome="reject";Code="unsupported-inner-foot";Diagnostics=[error.Message]}

    let add s g = addSelected s g (s.Program.Feet |> List.map _.Id |> Set.ofList)

    let ensure (s: ScreenedMotion) (g: WalkerGenome) =
        let eligible =
            g.Feet |> List.filter (fun choice ->
                let sole = g.Bodies |> List.collect _.Parts |> List.find (fun p -> p.Id=choice.Part)
                sole.Part="3701.dat" && (defaultArg choice.AdditionalContacts []).IsEmpty)
            |> List.map _.Foot |> Set.ofList
        if eligible.IsEmpty then Ok g else addSelected s g eligible
