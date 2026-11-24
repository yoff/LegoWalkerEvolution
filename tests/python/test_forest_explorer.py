import json
from pathlib import Path
import subprocess

from legowalkers_sim import proposal_forest


def run_node(code):
    script = Path(proposal_forest.__file__).with_name("forest_explorer.js")
    completed = subprocess.run(
        ["node", "-e", f"const f = require({json.dumps(str(script))});\n"
         'const assert = require("node:assert/strict");\n' + code],
        capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr


def test_folding_and_bidirectional_tracing_preserve_branches_and_exact_port_relations():
    run_node("""
const node = (key,parents=[],extra={})=>({key,parents,state:"pending",...extra});
const g=f.graph([
    node("seed"),node("other"),node("a",["seed"]),node("b",["a"]),
    node("winner",["b"],{state:"qualified"}),
    node("rejected",["a"],{state:"rejected"}),
    node("isolated",["other"]),node("other-winner",["isolated"],{state:"qualified"}),
    node("contribution",["a"],{contribution:true}),
    node("c",["contribution"]),node("current",["c"],{population:true}),
    node("reused",["winner"],{state:"qualified",reused:true}),
    node("merge",["b","c"]),node("merged-winner",["merge"],{state:"qualified"})
]);
const folded=f.fold(g,"qualified",new Set());
assert.deepEqual([...folded.inputs],["seed","other","contribution"]);
assert.deepEqual([...folded.outputs],["winner","other-winner","merged-winner"]);
assert.equal(folded.buckets.size,1);
assert.equal(new Set(folded.entities.map(e=>e.layer)).size,3);
const bucket=[...folded.buckets.values()][0];
assert(bucket.members.has("rejected"));
assert(bucket.ended.includes("rejected"));
assert.deepEqual([...f.trace(g,"winner","back","qualified")].sort(),["a","b","seed","winner"]);
assert.deepEqual([...f.trace(g,"seed","forward","qualified")].sort(),
    ["a","b","c","contribution","merge","merged-winner","seed","winner"]);
assert(!f.trace(g,"seed","forward","qualified").has("rejected"));
assert.deepEqual([...f.trace(g,"rejected","forward","qualified")],["rejected"]);
assert.deepEqual([...f.trace(g,"contribution","forward","population")].sort(),
    ["c","contribution","current"]);
assert.deepEqual([...f.fringe(g,"population").outputs],["current"]);
const incoming={direction:"forward",targets:["a"]};
const winner={direction:"back",targets:["b"]};
const unrelated={direction:"back",targets:["isolated"]};
assert.deepEqual(f.relatedPorts(g,bucket,incoming,[incoming,winner,unrelated]),[winner]);
assert.deepEqual(f.relatedPorts(g,bucket,winner,[incoming,winner,unrelated]),[incoming]);
const expanded=new Set(["a"]);
const step=f.fold(g,"qualified",expanded);
assert(step.visible.has("a"));
assert(![...step.buckets.values()].some(b=>b.members.has("a")));
assert(step.buckets.size>=1);
assert.deepEqual([...expanded],["a"]); // Folding does not rewrite user exploration.
for(const edge of step.edges) for(const [parent,child] of edge.pairs) {
    assert(g.byKey.get(child).parents.includes(parent));
}
assert.throws(()=>f.graph([node("x",["missing"])]),/Missing ancestry parent/);
assert.throws(()=>f.graph([node("x",["y"]),node("y",["x"])]),/cycle/);
assert.throws(()=>f.graph([node("x"),node("x")]),/Duplicate/);
assert.equal(f.fold(f.graph([]),"qualified",new Set()).entities.length,0);
""")


def test_large_graph_starts_with_three_columns_even_with_deep_contributions():
    run_node("""
const nodes=[{key:"root",parents:[],state:"pending"}];
for(let i=1;i<=2000;i++) nodes.push({
    key:"n"+i,parents:[i===1?"root":"n"+(i-1)],
    contribution:i%100===0,state:i===1999?"qualified":"pending"
});
const g=f.graph(nodes);
const folded=f.fold(g,"qualified",new Set());
assert.equal(folded.buckets.size,1);
assert.equal(new Set(folded.entities.map(e=>e.layer)).size,3);
assert.equal(folded.visible.size,22);
assert.equal([...folded.buckets.values()][0].members.size,1979);
const revealed=f.trace(g,"n1999","back","qualified");
assert.equal(revealed.size,2000);
assert.equal(g.nodes.length,2001);
""")


def test_grouped_pictures_and_names_identify_only_the_same_recorded_design():
    run_node("""
const nodes=[
    {key:"first",parents:[],label:"B1",state:"rejected",version:"v0001"},
    {key:"copy",parents:["first"],label:"B9",state:"pending",version:"v0002",
     contribution_name:"Walker2 outward L",contribution_href:"/workbench.html?node=saved",
     preview:"/versions/v0002/designs/exact/assembly-render/walker-v2.png",
     preview_caption:"Saved LEGO pose; not qualification."},
    {key:"repair",parents:["copy"],label:"B33",state:"rejected",
     preview:"/versions/v0002/designs/repair/candidate/cycle.svg",
     preview_caption:"Solved motion diagram; not live physics."},
    {key:"missing",parents:[],label:"B2",state:"pending"}
];
const g=f.graph(nodes),before=JSON.stringify(nodes),group={keys:["first","copy"],collapsed:true};
const view=f.presentation(g,g.byKey.get("first"),group);
assert.equal(view.name,"Walker2 outward L");
assert.equal(view.contributionHref,"/workbench.html?node=saved");
assert.equal(view.preview,nodes[1].preview);
assert.match(view.caption,/From v0002 B9/);
assert.equal(g.byKey.get("first").state,"rejected");
assert.equal(f.presentation(g,g.byKey.get("repair")).preview,nodes[2].preview);
assert.equal(f.presentation(g,g.byKey.get("missing")).preview,undefined);
assert.equal(JSON.stringify(nodes),before);
""")


def test_polling_additions_preserve_explicit_expansions_and_do_not_invent_outputs():
    run_node("""
const first=[{key:"root",parents:[],state:"pending"},
    {key:"a",parents:["root"],state:"pending"},
    {key:"b",parents:["a"],state:"pending"}];
const expanded=new Set(["a"]);
const before=f.fold(f.graph(first),"qualified",expanded);
assert.equal(before.outputs.size,0);
const after=f.fold(f.graph([...first,{key:"winner",parents:["b"],state:"qualified"}]),
    "qualified",expanded);
assert(after.visible.has("a"));
assert(after.visible.has("winner"));
assert(!after.visible.has("b"));
assert.deepEqual([...expanded],["a"]);
""")


def test_trace_to_ends_reveals_complete_terminal_paths_without_unrelated_ancestry():
    run_node("""
const make=(key,parents=[],extra={})=>({key,parents,state:"pending",...extra});
const id="a".repeat(32);
const g=f.graph([
    make("seed"),
    make("a1",["seed"],{contribution:true,contribution_id:id,version:"v0001"}),
    make("a2",["seed"],{contribution:true,contribution_id:id,version:"v0002"}),
    make("copy",["a2"],{carry_from:"a2",version:"v0003"}),
    make("middle",["copy"]),make("rejected",["middle"],{state:"rejected"}),
    make("pending",["copy"]),make("other"),make("winner",["other"],{state:"qualified"}),
    make("merge",["middle","other"])
]);
assert.deepEqual([...f.trace(g,"a2","forward","qualified")],["a2"]);
assert.deepEqual([...f.trace(g,"a2","ends","qualified")].sort(),
    ["a2","copy","merge","middle","pending","rejected"]);
assert(!f.trace(g,"a2","ends","qualified").has("other"));
assert.deepEqual([...f.trace(g,"rejected","ends","qualified")],["rejected"]);
const folded=f.fold(g,"qualified",new Set());
const members=folded.groups.get("a1").keys;
const trace=f.trace(g,members,"ends","qualified");
assert(trace.has("middle") && trace.has("rejected") && trace.has("pending"));
const revealed=f.fold(g,"qualified",trace);
assert(revealed.visible.has("middle") && revealed.visible.has("merge"));
assert(!revealed.visible.has("copy"));
assert.equal(revealed.aliases.get("copy"),"a1");
assert.equal(g.byKey.get("pending").state,"pending");
""")


def test_repeat_admissions_use_first_identity_and_retain_version_specific_edges_and_traces():
    run_node("""
const make=(key,parents=[],extra={})=>({key,parents,state:"pending",...extra});
const id="a".repeat(32);
const original=[
    make("seed1"),make("seed2"),
    make("bv0002:4",["seed2"],{contribution:true,contribution_id:id,version:"v0002"}),
    make("bv0001:9",["seed1"],{contribution:true,contribution_id:id,version:"v0001"}),
    make("branch",["bv0002:4"]),make("winner",["branch"],{state:"qualified"}),
    make("distinct",["seed2"],{contribution:true,contribution_id:"b".repeat(32)}),
    make("legacy",["seed2"],{contribution:true})
];
const g=f.graph(original), before=JSON.stringify(original);
const folded=f.fold(g,"qualified",new Set());
assert.equal(folded.compressed,1);
assert(folded.inputs.has("bv0001:9"));
assert(!folded.inputs.has("bv0002:4"));
assert(folded.inputs.has("distinct") && folded.inputs.has("legacy"));
assert.equal(folded.aliases.get("bv0002:4"),"bv0001:9");
assert(!folded.entities.some(e=>e.key==="bv0002:4"));
assert(![...folded.buckets.values()].some(b=>b.members.has("bv0002:4")));
assert.equal(folded.groups.get("bv0001:9").keys.length,2);
const pairs=folded.edges.flatMap(e=>e.pairs);
assert(pairs.some(([p,c])=>p==="seed2" && c==="bv0002:4"));
assert(pairs.some(([p,c])=>p==="bv0002:4" && c==="branch"));
assert.deepEqual([...f.trace(g,"seed1","forward","qualified")],["seed1"]);
assert(!f.trace(g,"winner","back","qualified").has("bv0001:9"));
assert.deepEqual([...f.trace(g,["bv0001:9","bv0002:4"],"forward","qualified")].sort(),
    ["branch","bv0002:4","winner"]);
const separate=new Set([id]), expanded=f.fold(g,"qualified",new Set(),separate);
assert.equal(expanded.compressed,0);
assert(expanded.inputs.has("bv0002:4"));
assert.equal(expanded.groups.get("bv0001:9").collapsed,false);
assert.equal(JSON.stringify(original),before);
assert.deepEqual([...separate],[id]);
const updated=f.graph([...original,make("bv0003:1",["seed2"],
    {contribution:true,contribution_id:id,version:"v0003"})]);
const next=f.fold(updated,"qualified",new Set(),separate);
assert(next.visible.has("bv0003:1"));
assert.equal(next.groups.get("bv0001:9").keys.length,3);
assert.equal(f.fold(updated,"qualified",new Set()).compressed,2);
""")


def test_carry_forward_chains_and_forks_collapse_only_recorded_identity_links():
    run_node("""
const make=(key,parents=[],extra={})=>({key,parents,state:"pending",...extra});
const original=[
    make("seed"),
    make("source",["seed"],{version:"v0001"}),
    make("copy1",["source"],{version:"v0002",carry_from:"source"}),
    make("copy2",["copy1"],{version:"v0003",carry_from:"copy1",population:true}),
    make("fork-copy",["source"],{version:"v0004",carry_from:"source"}),
    make("repair",["copy2"],{version:"v0003",operation:"repair-realization"}),
    make("winner",["repair"],{version:"v0003",state:"qualified"}),
    make("only-a-label",["source"],{operation:"carry forward"})
];
const g=f.graph(original), before=JSON.stringify(original);
const folded=f.fold(g,"qualified",new Set());
assert.equal(folded.compressed,3);
const group=folded.groups.get("source");
assert.equal(group.kind,"carry-forward copies");
assert.equal(group.carries,3);
assert.deepEqual(group.keys,["source","copy1","copy2","fork-copy"]);
assert.equal(folded.aliases.get("copy2"),"source");
assert.equal(folded.aliases.get("repair"),"repair");
assert.equal(folded.aliases.get("only-a-label"),"only-a-label");
const entities=new Set(folded.entities.map(e=>e.key));
assert(folded.edges.every(edge=>entities.has(edge.from)&&entities.has(edge.to)));
const bucket=[...folded.buckets.values()][0];
assert(bucket.members.has("source"));
assert(!bucket.members.has("copy2"));
assert(bucket.traversal.has("copy2"));
const input={direction:"forward",targets:["source"]}, output={direction:"back",targets:["repair"]};
assert.deepEqual(f.relatedPorts(g,bucket,input,[input,output]),[output]);
assert.deepEqual([...f.trace(g,"winner","back","qualified")].sort(),
    ["copy1","copy2","repair","seed","source","winner"]);
const population=f.fold(g,"population",new Set());
assert.deepEqual([...population.outputs],["source"]);
assert.equal(population.entities.find(e=>e.key==="source").node.state,"pending");
const expanded=f.fold(g,"population",new Set(),new Set([group.id]));
for(const key of group.keys) assert(expanded.visible.has(key));
assert.equal(expanded.compressed,0);
assert(expanded.outputs.has("copy2"));
const hinted=f.fold(g,"qualified",new Set(["copy2"]));
assert(hinted.visible.has("source"));
assert(!hinted.visible.has("copy2"));
assert.equal(JSON.stringify(original),before);
assert.throws(()=>f.versionGroups(f.graph([make("a"),make("b"),make("bad",["a"],
    {carry_from:"b"})])),/Invalid carry-forward provenance/);
""")


def test_carry_copies_and_repeat_admissions_share_one_expandable_identity_group():
    run_node("""
const id="a".repeat(32);
const g=f.graph([
    {key:"seed",parents:[],state:"pending"},
    {key:"a1",parents:["seed"],state:"pending",contribution:true,contribution_id:id,version:"v0001"},
    {key:"carry1",parents:["a1"],state:"pending",carry_from:"a1",version:"v0002"},
    {key:"a2",parents:["seed"],state:"pending",contribution:true,contribution_id:id,version:"v0003"},
    {key:"carry2",parents:["a2"],state:"pending",carry_from:"a2",version:"v0004",population:true}
]);
const folded=f.fold(g,"population",new Set());
assert.equal(folded.compressed,3);
const group=folded.groups.get("a1");
assert.equal(group.id,id);
assert.equal(group.kind,"version copies");
assert.equal(group.admissions,2);
assert.equal(group.carries,2);
assert.equal(folded.aliases.get("carry2"),"a1");
assert.deepEqual([...folded.outputs],["a1"]);
const expanded=f.fold(g,"population",new Set(),new Set([id]));
assert.equal(expanded.visible.size,5);
assert.equal(expanded.compressed,0);
""")


def test_explorer_dom_actions_keyboard_ports_undo_reset_and_live_updates():
    script = Path(proposal_forest.__file__).with_name("forest_explorer.js")
    run_node("""
class Element {
    constructor(tag) {
        this.tagName=tag;this.children=[];this.dataset={};this.attributes={};this.listeners={};
        this.textContent="";this.className="";this.scrollLeft=0;this.scrollTop=0;this.disabled=false;
    }
    append(...items) {for(const item of items) {item.parentElement=this;this.children.push(item);}}
    replaceChildren(...items) {this.children=[];this.append(...items);}
    setAttribute(key,value) {
        this.attributes[key]=String(value);
        if(key==="class")this.className=value;
        if(key==="data-forest-focus")this.dataset.forestFocus=value;
    }
    matches(selector) {
        return selector.startsWith(".")?this.className.split(" ").includes(selector.slice(1)):
            selector==="[data-forest-focus]"?!!this.dataset.forestFocus:this.tagName===selector;
    }
    querySelectorAll(selector) {
        return this.children.flatMap(child=>[
            ...(child.matches(selector)?[child]:[]),...child.querySelectorAll(selector)]);
    }
    querySelector(selector) {return this.querySelectorAll(selector)[0];}
    closest(selector) {return this.matches(selector)?this:this.parentElement?.closest(selector);}
    addEventListener(type,listener) {this.listeners[type]=listener;}
    fire(type,extra={}) {if(!this.disabled)this.listeners[type]?.({preventDefault(){},...extra});}
    focus() {document.activeElement=this;this.fire("focus");}
    scrollIntoView() {this.scrolled=true;}
    get isConnected() {return true;}
}
global.window={};
const documentElement=new Element("html");
global.document={documentElement,activeElement:null,
    createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag),
    querySelectorAll:selector=>documentElement.querySelectorAll(selector)};
let mutation;
global.MutationObserver=class {constructor(callback){mutation=callback;} observe(){}};
const panel=new Element("section");panel.className="proposal-forest";
const root=new Element("div");root.className="forest-explorer";
const data=new Element("script");data.className="forest-data";
const full=new Element("details");full.className="forest-full";
panel.append(root,data,full);documentElement.append(panel);
const make=(key,parents=[],extra={})=>({key,parents,label:key,operation:"mutation",
    detail:"Recorded "+key,href:"history/"+key+".html",state:"pending",...extra});
const nodes=[make("seed"),make("a",["seed"]),make("winner",["a"],{state:"qualified"}),
    make("ended",["a"],{state:"rejected"}),make("current",["a"],{population:true})];
data.textContent=JSON.stringify(nodes);
""" + f'require("node:vm").runInThisContext(require("node:fs").readFileSync({json.dumps(str(script))},"utf8"));\n' + """
function click(label) {
    const b=root.querySelectorAll("button").find(node=>node.textContent===label);
    assert(b,"Missing action: "+label);b.fire("click");return b;
}
function select(key) {
    const n=root.querySelectorAll(".fold-node").find(node=>node.dataset.forestFocus===key);
    assert(n,"Missing visible node: "+key);n.fire("click");
}
assert(root.querySelector("p").textContent.startsWith("2 visible / 5"));
assert(root.querySelectorAll(".fold-port").length===2);
const input=root.querySelectorAll(".fold-port").find(p=>p.attributes["aria-label"].includes("descendants"));
input.focus();
assert.equal(root.querySelectorAll(".fold-port").filter(p=>p.attributes["data-highlight"]==="true").length,2);
input.fire("keydown",{key:"Enter"});
assert(root.querySelector("p").textContent.startsWith("3 visible / 5"));
assert.equal(document.activeElement.dataset.forestFocus,"a");
assert(root.querySelectorAll("a").some(link=>link.href==="history/a.html"));
click("Undo expansion");
assert(root.querySelector("p").textContent.startsWith("2 visible / 5"));
select("seed");click("Trace to outputs");
assert(root.querySelector("p").textContent.startsWith("3 visible / 5"));
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="ended"));
click("Show ended branches (2)");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="ended"));
click("Reset overview");
select("winner");click("Trace to origins");
assert(root.querySelector("p").textContent.startsWith("3 visible / 5"));
const mode=root.querySelector("select");mode.value="population";mode.fire("change");
assert.equal(root.querySelector("select").value,"population");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="current"));
root.querySelector(".forest-map").scrollLeft=137;
root.querySelector(".forest-map").scrollTop=83;
const oldRoot=root;
data.textContent=JSON.stringify([...nodes,make("new",["current"])]);
mutation();
assert.equal(root,oldRoot);
assert.equal(root.querySelector("select").value,"population");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="a"));
assert.equal(root.querySelector(".forest-map").scrollLeft,137);
assert.equal(root.querySelector(".forest-map").scrollTop,83);
click("Full forest");assert.equal(full.open,true);
select("a");click("Collapse here");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="a"));
data.textContent="not JSON";mutation();
assert.equal(root.children[0].attributes.role,"alert");
assert.match(root.children[0].textContent,/Compact ancestry unavailable/);
const warning=root.children[0];mutation();
assert.equal(root.children[0],warning);
const cid="a".repeat(32);
const repeated=[make("seed"),make("c1",["seed"],
    {version:"v0001",contribution:true,contribution_id:cid,
     contribution_name:"Walker2 outward L",contribution_href:"/workbench.html?node=exact"}),
    make("c2",["seed"],{version:"v0002",contribution:true,contribution_id:cid,
     preview:"/versions/v0002/designs/exact/assembly-render/walker-v2.png",
     preview_caption:"Saved LEGO pose; not qualification."}),
    make("x",["c2"]),make("winner",["x"],{state:"qualified"})];
data.textContent=JSON.stringify(repeated);mutation();click("Reset overview");
const qualifiedMode=root.querySelector("select");qualifiedMode.value="qualified";qualifiedMode.fire("change");
assert(root.querySelector("p").textContent.startsWith("3 visible / 5"));
assert.match(root.querySelector("p").textContent,/1 repeated contribution admissions grouped/);
const finder=root.querySelectorAll("select").find(s=>s.attributes["aria-label"]==="Find contribution by name");
assert(finder);
assert(finder.querySelectorAll("option").some(option=>option.textContent.includes("Walker2 outward L")));
finder.value="c1";finder.fire("change");
assert(root.querySelector(".forest-inspector").scrolled);
assert(root.querySelectorAll("h3").some(h=>h.textContent==="Walker2 outward L"));
assert(root.querySelectorAll("a").some(a=>a.href==="/workbench.html?node=exact"));
const preview=root.querySelector("image");
assert.equal(preview.attributes.href,"/versions/v0002/designs/exact/assembly-render/walker-v2.png");
assert.match(preview.attributes["aria-label"],/From v0002 c2/);
preview.fire("error");
assert.equal(preview.attributes.visibility,"hidden");
assert(root.querySelectorAll("text").some(t=>t.textContent==="Saved preview unavailable."));
select("c1");
assert(root.querySelectorAll("p").some(p=>p.textContent.includes("not a combined trial result")));
click("Expand version admissions (2)");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c2"));
assert(root.querySelectorAll("a").some(a=>a.href==="history/c2.html"));
const modeAgain=root.querySelector("select");modeAgain.value="population";modeAgain.fire("change");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c2"));
data.textContent=JSON.stringify([...repeated,make("c3",["seed"],
    {version:"v0003",contribution:true,contribution_id:cid})]);mutation();
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c3"));
select("c2");click("Collapse version admissions (3)");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c2"));
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c3"));
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c1"));
click("Undo expansion");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c3"));
click("Reset overview");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="c2"));
const carried=[make("seed"),make("source",["seed"],{version:"v0001"}),
    make("copy1",["source"],{version:"v0002",carry_from:"source"}),
    make("copy2",["copy1"],{version:"v0003",carry_from:"copy1",population:true}),
    make("fork",["source"],{version:"v0004",carry_from:"source"}),
    make("winner",["copy2"],{state:"qualified"})];
data.textContent=JSON.stringify(carried);mutation();click("Reset overview");
assert(root.querySelector("p").textContent.startsWith("2 visible / 6"));
select("source");
assert(root.querySelectorAll("p").some(p=>/Output membership.*copy2/.test(p.textContent)));
click("Expand carry-forward copies (4)");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="copy2"));
assert(root.querySelectorAll("a").some(a=>a.href==="history/copy1.html"));
data.textContent=JSON.stringify([...carried,make("copy3",["copy2"],
    {version:"v0005",carry_from:"copy2"})]);mutation();
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="copy3"));
select("copy2");click("Collapse carry-forward copies (5)");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="copy2"));
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="source"));
click("Undo expansion");
assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="copy3"));
click("Reset overview");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="copy1"));
const ended=[make("seed"),make("mid1",["seed"]),make("mid2",["mid1"]),
    make("rejected",["mid2"],{state:"rejected"}),make("pending",["mid1"]),
    make("other"),make("winner",["other"],{state:"qualified"})];
data.textContent=JSON.stringify(ended);mutation();click("Reset overview");
const endsMode=root.querySelector("select");endsMode.value="qualified";endsMode.fire("change");
select("seed");
assert(!root.querySelectorAll("button").some(b=>b.textContent==="Trace to outputs"));
assert.equal(root.querySelectorAll("button").find(b=>b.textContent==="Trace to ends").disabled,false);
click("Trace to ends");
for(const key of ["mid1","mid2","rejected","pending"]) {
    assert(root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus===key));
}
assert.equal(root.querySelectorAll("button").find(b=>b.textContent==="Trace to ends").disabled,true);
click("Undo expansion");
assert(!root.querySelectorAll(".fold-node").some(n=>n.dataset.forestFocus==="mid1"));
click("Trace to ends");
select("rejected");
assert.equal(root.querySelectorAll("button").find(b=>b.textContent==="Trace to ends").disabled,true);
select("seed");
data.textContent=JSON.stringify([...ended,make("qualified",["mid2"],{state:"qualified"})]);mutation();
assert(root.querySelectorAll("button").some(b=>b.textContent==="Trace to outputs"));
assert(!root.querySelectorAll("button").some(b=>b.textContent==="Trace to ends"));
const noPopulation=root.querySelector("select");noPopulation.value="population";noPopulation.fire("change");
assert(root.querySelectorAll("button").some(b=>b.textContent==="Trace to ends"));
assert(!root.querySelectorAll("button").some(b=>b.textContent==="Trace to outputs"));
""")
