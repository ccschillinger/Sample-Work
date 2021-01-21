"use strict";

// RPC wrapper
function invoke_rpc(method, args, timeout, on_done){
  $("#crash").hide();
  $("#timeout").hide();
  $("#rpc_spinner").show();
  //send RPC with whatever data is appropriate. Display an error message on crash or timeout
  var xhr = new XMLHttpRequest();
  xhr.open("POST", method, true);
  xhr.setRequestHeader('Content-Type','application/json; charset=UTF-8');
  xhr.timeout = timeout;
  xhr.send(JSON.stringify(args));
  xhr.ontimeout = function () {
    $("#timeout").show();
    $("#rpc_spinner").hide();
    $("#crash").hide();
  };
  xhr.onloadend = function() {
    if (xhr.status === 200) {
      $("#rpc_spinner").hide();
      var result = JSON.parse(xhr.responseText)
      $("#timeout").hide();
      if (typeof(on_done) != "undefined"){
        on_done(result);
      }
    } else {
      $("#crash").show();
    }
  }
}

// Resource load wrapper
function load_resource(name, on_done) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", name, true);
  xhr.onloadend = function() {
    if (xhr.status === 200) {
      var result = JSON.parse(xhr.responseText);
      on_done(result);
    }
  }
  xhr.send();
}

// Code that runs first
$(document).ready(function(){
    // race condition if init() does RPC on function not yet registered by restart()!
    //restart();
    //init();
    invoke_rpc( "/restart", {}, 0, function() { init(); } )
});

function restart(){
  invoke_rpc( "/restart", {} )
}

function handle_query(){
  let query = document.getElementById("query").value;
  let graphClass = document.getElementById("graphClass").value;

  let query_handler = function( result ) {
    if (typeof result === "string") {
      $("#message_out").text(result).addClass("bad");
    } else {
      $("#message_out").text("Query result shown!").removeClass("bad");
      updateGraph(result[0], result[1]);
    }
  };

  let args = { "query": query, "graphClass": graphClass };
  invoke_rpc( "/query", args, 1000, query_handler);
}

function init(){
  
  $('#query').immybox({
    choices: [
      {text: '[("a", [1,2]), ("b", [0,2]), ("c", [0,1])]', value: '[("a", [1,2]), ("b", [0,2]), ("c", [0,1])]'},
      {text: '[("*", [])]', value: '[("*", [])]'},
      {text: '[("*", [1,2,3,4]), ("*", [0,2,3,4]), ("*", [0,1,3,4]), ("*", [0,1,2,4]), ("*", [0,1,2,3])]', value: '[("*", [1,2,3,4]), ("*", [0,2,3,4]), ("*", [0,1,3,4]), ("*", [0,1,2,4]), ("*", [0,1,2,3])]'},
      {text: '[("a", [1]), ("*", [0])]', value: '[("a", [1]), ("*", [0])]'},
      {text: '[("h", [])]', value: '[("h", [])]'},
      {text: '[("*", [1,2]), ("*", [0,2]), ("*", [0,1])]', value: '[("*", [1,2]), ("*", [0,2]), ("*", [0,1])]'},
    
    ]
  });

  d3.json("/resources/ui/ui_graph.json", function (error, graph) {
    if (error) throw error;
    let forceGraph = {}

    // Create nodes of the force graph
    forceGraph["nodes"] = []
    forceGraph["links"] = []
    for (let key in graph) {
      forceGraph["nodes"].push(
        { "name": parseInt(key),
          "value": graph[key][0],
          "group": 1
        })

      // Add force graph links for this node
      for (let target in graph[key][1]) {
        forceGraph["links"].push({
          "source": parseInt(key),
          "target": graph[key][1][target],
          "value": 1,
          "edge_color": "#999"
        })
      }
    }

    window.graph = forceGraph;
    drawForce(forceGraph);
  });
}

// D3-related mess
function getColor(groupNumber){
  switch(groupNumber) {
    case 1:
      return "#aec7e8";
    case 2:
      return "#ff7f0e";
    case 3:
      return "#1f77b4";
    case 4:
      return "#2ca02c"
    default:
      return "#aec7e8";
  }
}

function getValuebyId(val){
  if (window.graph) {
    return window.graph.nodes[val].name;
  } else { return null; }
}

var force;

function handle_resize(){
  var width = document.getElementById('graph').offsetWidth;
  var height = document.getElementById('graph').offsetHeight;
  force.size([width, height]).resume();
}
window.onresize = handle_resize;

function drawForce(graph) {
  var svg = d3.select("#graph")
              .html('')
              .append("svg")
              .attr("width", "100%")
              .attr("height", "80ex");
              

  force = d3.layout.force()
                    .charge(-300)
                    .linkDistance(100);

  handle_resize();

  svg.append('defs').append('marker')
        .attr('id', 'arrowhead')
        .attr('viewBox', '-0 -5 10 10')
        .attr('refX', 26)
        .attr('refY', 0)
        .attr('orient', 'auto')
        .attr('markerWidth', 10)
        .attr('markerHeight', 10)
        .attr('xoverflow', 'visible')
        .append('svg:path')
        .attr('d', 'M 0,-5 L 10 ,0 L 0,5')
        .attr('fill', '#999')
        .style('stroke','none');

  svg.append('defs').append('marker')
        .attr('id', 'red-arrowhead')
        .attr('viewBox', '-0 -5 10 10')
        .attr('refX', 26)
        .attr('refY', 0)
        .attr('orient', 'auto')
        .attr('markerWidth', 10)
        .attr('markerHeight', 10)
        .attr('xoverflow', 'visible')
        .append('svg:path')
        .attr('d', 'M 0,-5 L 10 ,0 L 0,5')
        .attr('fill', '#f00')
        .style('stroke','none');

  force.nodes(graph.nodes)
       .links(graph.links)
       .start();

  var link = svg.selectAll(".link")
                .data(graph.links)
                .enter().append("line")
                .attr("class", "link")
                .attr('marker-end', function(d) {
                  if (d.edge_color == '#f00') {
                    return 'url(#red-arrowhead)'
                  } else {
                    return 'url(#arrowhead)'
                  }
                })
                .attr('stroke', function (d) {
                  return d.edge_color;
                })
                .style("stroke-width", function (d) {
                  return Math.sqrt(d.value);
                });                

  var node = svg.selectAll(".node")
                .data(graph.nodes)
                .enter().append("circle")
                .attr("class", "node")
                .attr("r", 16)
                .style("fill", function (d) {
                  return getColor(d.group);//color(d.group);
                })
                .call(force.drag);

  node.append("title") .text(function (d) { return d.name; });
  
  var label = svg.selectAll(".mytext")
						.data(graph.nodes)
						.enter()
						.append("text")
					    .text(function (d) { return d.name + ":" + ( (d.value != '') ? d.value : "''"); })
					    .style("text-anchor", "middle")
					    .style("fill", "#555")
              .style("font-family", "Arial")
              .style("pointer-events", "none")
					    .style("font-size", 12);

  force.on("tick", function () {
    link.attr("x1", function (d) {
          return d.source.x;
        })
       .attr("y1", function (d) {
          return d.source.y;
       })
       .attr("x2", function (d) {
          return d.target.x;
       })
       .attr("y2", function (d) {
          return d.target.y;
       });

       node.attr("cx", function (d) { return d.x; })
           .attr("cy", function (d) { return d.y; });

      label.attr("x", function(d){ return d.x; })
      .attr("y", function (d) {return d.y + 5; });
  });
}

function updateGraph(queryResults, pattern) {
  if (window.graph) {
    var graph = window.graph;

    // reset the graph
    // clear selected nodes
    for (var i = 0; i < graph.nodes.length; i++) {
      graph.nodes[i].group = 1;
    }
    // clear selected edges
    for (var i = 0; i < graph.links.length; i++) {
      graph.links[i].edge_color = "#999999";
      graph.links[i].value = 1;
    }


    for (let values of queryResults) {
      // Select all nodes in values
      for (var i = 0; i < graph.nodes.length; i++) {
        // if node is in values, select it
        var g_val = graph.nodes[i].name;
        if (values.indexOf(g_val) >= 0) {
          graph.nodes[i].group = 4;
        }
      }

      // Select all edges defined in pattern and in values
      for (let i in pattern) {
        for (let j in pattern[i][1]){
          let x = graph.nodes[values[i]]
          let y = graph.nodes[values[pattern[i][1][j]]]

          for (var k = 0; k < graph.links.length; k++) {
            // figure out what this edge connects
            let xLink, yLink;
            if (typeof graph.links[k].source == "object"){
              xLink = graph.links[k].source.name;
              yLink = graph.links[k].target.name;
            } else {
              xLink = getValuebyId(graph.links[k].source);
              yLink = getValuebyId(graph.links[k].target);
            }

            // If this edge connects something defined by the pattern, color it red
            if ((xLink == x.name) && (yLink == y.name)) {
              graph.links[k].edge_color = "#f00";
            }
          }
        }
      }
    }

    drawForce(graph);
  }
}

