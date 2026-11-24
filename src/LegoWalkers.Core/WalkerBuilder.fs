namespace LegoWalkers

open System
open System.IO
open System.Security.Cryptography
open System.Text
open System.Text.Json
open System.Text.Json.Serialization

type WalkerBuilderPolicy = {
    SchemaVersion: int
    MotifOrder: string list
    PinOrder: string list
    LayerOffsets: int list
    ClearanceFirst: bool
}

type WalkerConstructionMotif = {
    Id: string
    Version: string
    Origin: string
    Evidence: string list
    Parameters: string list
    Preconditions: string list
    Preserves: string list
    Limitations: string list
}

module WalkerBuilder =
    let policyVersion = "symbolic-builder-policy-v1"
    let catalogueVersion = "technic-construction-catalogue-v2"
    let catalogue = [
        { Id = "smooth-passive-pivot"; Version = "2"; Origin = "hand-authored"
          Evidence = ["output/realization-quality-v3: rail/crank-pin interference recovered by shorter pin."]
          Parameters = ["Pin: 32002.dat / 3673.dat / 32556.dat"; "Original or reversed orientation"]
          Preconditions = ["Frictionless pivot"; "Exact declared-body engagement"
                           "Supported connector contact capacity"; "Full modeled socket-wall coverage"
                           "Whole-walker compiler accepts"]
          Preserves = ["Motion"; "Structural placements"; "Shaft midpoint"; "Motor settings"; "Feet"]
          Limitations = ["Contact capacity follows the supported smooth-pin mesh lengths; exact reconstruction remains mandatory."
                         "Modeled socket-wall coverage is not proof of axial retention."
                         "Clearance score is a conservative time-zero heuristic, not a collision test."] }
        { Id = "passive-link-layer-clearance"; Version = "2"; Origin = "hand-authored"
          Evidence = ["output/realization-quality-v3: unintended shaft/coupler rigid merge recovered by +10 LDU shift."]
          Parameters = ["Outward-relative depth offsets: +10 / -10 / +20 / -20 LDU"]
          Preconditions = ["Feedback implicates a passive link"; "No driven/chassis body or rigid mount"
                           "Only smooth incident pivots"; "Exact incident-pivot membership"
                           "Full modeled socket-wall coverage and supported contact capacity with existing pins"
                           "Whole-walker compiler accepts"]
          Preserves = ["Planar motion"; "Part rotations"; "Pins"; "Motor settings"; "Foot attachment with updated depth"]
          Limitations = ["Known axial envelopes only."
                         "Not a spacer, axial-retention, strength or assembly-order certificate."] }
    ]
    let defaultPolicy = {
        SchemaVersion = 1
        MotifOrder = catalogue |> List.map _.Id
        PinOrder = ["32002.dat";"3673.dat";"32556.dat"]
        LayerOffsets = [10;-10;20;-20]
        ClearanceFirst = true
    }

    let validate policy =
        let permutation label expected actual =
            if isNull (box actual) || List.length actual <> List.length expected
               || Set.ofList actual <> Set.ofList expected then
                invalidArg "builder-policy" $"{label} must be a permutation of the supported catalogue values."
        if isNull (box policy) || policy.SchemaVersion <> 1 then
            invalidArg "builder-policy" "Expected builder policy schema version one."
        permutation "motifOrder" defaultPolicy.MotifOrder policy.MotifOrder
        permutation "pinOrder" defaultPolicy.PinOrder policy.PinOrder
        permutation "layerOffsets" defaultPolicy.LayerOffsets policy.LayerOffsets
        policy

    let readPolicy path =
        let options = JsonSerializerOptions(PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                                            UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
                                            RespectRequiredConstructorParameters = true)
        JsonSerializer.Deserialize<WalkerBuilderPolicy>(File.ReadAllText path, options) |> validate

    let policyId policy =
        validate policy |> Realization.json |> Encoding.UTF8.GetBytes |> SHA256.HashData
        |> Convert.ToHexString |> _.ToLowerInvariant()

    let describe policy =
        let policy = validate policy
        {| schemaVersion = 1; policyVersion = policyVersion; policyId = policyId policy; policy = policy
           catalogueVersion = catalogueVersion; catalogue = catalogue
           tieBreak = "Canonical pivot/body ID, configured parameter order, then orientation."
           validation = "Policy orders supported actions; it cannot change hard checks or caller budgets."
           promotion = "Manual review required; no automatic catalogue or live-policy promotion." |}

    let write directory policy =
        File.WriteAllText(Path.Combine(directory, "builder.json"), Realization.json (describe policy)+"\n")
