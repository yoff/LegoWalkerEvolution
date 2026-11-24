#load "PackagingRealizationSupport.fsx"

open System
open PackagingRealizationSupport

match fsi.CommandLineArgs |> Array.skip 1 |> Array.toList with
| ["--input";input;"--plans";plans;"--plan";plan;"--output";output;"--limit";limit] ->
    generate input plans plan output (Int32.Parse limit)
| ["--repair-parent";parent;"--feedback";feedback;"--output";output;"--limit";limit] ->
    repair parent feedback output (Int32.Parse limit)
| _ -> invalidArg "arguments" "Expected --input COMBINED_INPUT --plans COMBINED_PLAN --plan PLAN_ID --output FRESH_DIR --limit 1|2, or --repair-parent CANDIDATE --feedback FEEDBACK --output FRESH_DIR --limit 1..32."
