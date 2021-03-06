// Copyright 2010-2021 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// A collections of i/o utilities for the Graph classes in ./graph.h.

#ifndef UTIL_GRAPH_IO_H_
#define UTIL_GRAPH_IO_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "ortools/base/filelineiter.h"
#include "ortools/graph/graph.h"

namespace util {

// Returns a string representation of a graph.
enum GraphToStringFormat {
  // One arc per line, eg. "3->1".
  PRINT_GRAPH_ARCS,

  // One space-separated adjacency list per line, eg. "3: 5 1 3 1".
  // Nodes with no outgoing arc get an empty list.
  PRINT_GRAPH_ADJACENCY_LISTS,

  // Ditto, but the adjacency lists are sorted.
  PRINT_GRAPH_ADJACENCY_LISTS_SORTED,
};
template <class Graph>
std::string GraphToString(const Graph& graph, GraphToStringFormat format);

// Read a graph file in the simple ".g" format: the file should be a text file
// containing only space-separated integers, whose first line is:
//   <num nodes> <num edges> [<num_colors> <index of first node with color #1>
//                            <index of first node with color #2> ...]
// and whose subsequent lines represent edges if "directed" is false, or arcs if
// "directed" is true:
//   <node1> <node2>.
//
// This returns a newly created graph upon success, which the user needs to take
// ownership of, or a failure status. See absl/status/statusor.h.
//
// If "num_nodes_with_color_or_null" is not nullptr, it will be filled with the
// color information: num_nodes_with_color_or_null[i] will be the number of
// nodes with color #i. Furthermore, nodes are sorted by color.
//
// Examples:
//   // Simply crash if the graph isn't successfully read from the file.
//   typedef StaticGraph<> MyGraph;  // This is just an example.
//   std::unique_ptr<MyGraph> my_graph(
//       ReadGraphFile<MyGraph>("graph.g", /*directed=*/ false).ValueOrDie());
//
//   // More complicated error handling.
//   absl::StatusOr<MyGraph*> error_or_graph =
//       ReadGraphFile<MyGraph>("graph.g", /*directed=*/ false);
//   if (!error_or_graph.ok()) {
//     LOG(ERROR) << "Error: " << error_or_graph.status().error_message();
//   } else {
//     std::unique_ptr<MyGraph> my_graph(error_or_graph.ValueOrDie());
//     ...
//   }
template <class Graph>
absl::StatusOr<Graph*> ReadGraphFile(
    const std::string& filename, bool directed,
    std::vector<int>* num_nodes_with_color_or_null);

// Writes a graph to the ".g" file format described above. If "directed" is
// true, all arcs are written to the file. If it is false, the graph is expected
// to be undirected (i.e. the number of arcs a->b is equal to the number of arcs
// b->a for all nodes a,b); and only the arcs a->b where a<=b are written. Note
// however that in this case, the symmetry of the graph is not fully checked
// (only the parity of the number of non-self arcs is).
//
// "num_nodes_with_color" is optional. If it is not empty, then the color
// information will be written to the header of the .g file. See ReadGraphFile.
//
// This method is the reverse of ReadGraphFile (with the same value for
// "directed").
template <class Graph>
absl::Status WriteGraphToFile(const Graph& graph, const std::string& filename,
                              bool directed,
                              const std::vector<int>& num_nodes_with_color);

// Implementations of the templated methods.

template <class Graph>
std::string GraphToString(const Graph& graph, GraphToStringFormat format) {
  std::string out;
  std::vector<typename Graph::NodeIndex> adj;
  for (const typename Graph::NodeIndex node : graph.AllNodes()) {
    if (format == PRINT_GRAPH_ARCS) {
      for (const typename Graph::ArcIndex arc : graph.OutgoingArcs(node)) {
        if (!out.empty()) out += '\n';
        absl::StrAppend(&out, node, "->", graph.Head(arc));
      }
    } else {  // PRINT_GRAPH_ADJACENCY_LISTS[_SORTED]
      adj.clear();
      for (const typename Graph::ArcIndex arc : graph.OutgoingArcs(node)) {
        adj.push_back(graph.Head(arc));
      }
      if (format == PRINT_GRAPH_ADJACENCY_LISTS_SORTED) {
        std::sort(adj.begin(), adj.end());
      }
      if (node != 0) out += '\n';
      absl::StrAppend(&out, node, ": ", absl::StrJoin(adj, " "));
    }
  }
  return out;
}

template <class Graph>
absl::StatusOr<Graph*> ReadGraphFile(
    const std::string& filename, bool directed,
    std::vector<int>* num_nodes_with_color_or_null) {
  std::unique_ptr<Graph> graph;
  int64_t num_nodes = -1;
  int64_t num_expected_lines = -1;
  int64_t num_lines_read = 0;
  for (const std::string& line : FileLines(filename)) {
    ++num_lines_read;
    if (num_lines_read == 1) {
      std::vector<int64_t> header_ints;
      // if (!SplitStringAndParse(line, " ", &strings::safe_strto64,
      //                          &header_ints) ||
      //     header_ints.size() < 2 || header_ints[0] < 0 || header_ints[1] < 0)
      //     {
      //        return absl::Status(
      //            absl::StatusCode::kInvalidArgument,
      //       absl::StrCat("First line of '", filename,
      //                    "' should be at least two nonnegative integers."));
      // }
      num_nodes = header_ints[0];
      num_expected_lines = header_ints[1];
      if (num_nodes_with_color_or_null != nullptr) {
        num_nodes_with_color_or_null->clear();
        if (header_ints.size() == 2) {
          // No coloring: all the nodes have the same color.
          num_nodes_with_color_or_null->push_back(num_nodes);
        } else {
          const int num_colors = header_ints[2];
          if (header_ints.size() != num_colors + 2) {
            return absl::Status(
                absl::StatusCode::kInvalidArgument,
                absl::StrFormat(
                    "There should be num_colors-1 color cardinalities in the"
                    " header of '%s' (where num_colors=%d): the last color"
                    " cardinality should be skipped",
                    filename, num_colors));
          }
          num_nodes_with_color_or_null->reserve(num_colors);
          int num_nodes_left = num_nodes;
          for (int i = 3; i < header_ints.size(); ++i) {
            num_nodes_with_color_or_null->push_back(header_ints[i]);
            num_nodes_left -= header_ints[i];
            if (header_ints[i] <= 0 || num_nodes_left <= 0) {
              return absl::Status(
                  absl::StatusCode::kInvalidArgument,
                  absl::StrFormat("The color cardinalities in the header of"
                                  " '%s' should always be >0 and add up to less"
                                  " than the total number of nodes",
                                  filename));
            }
          }
          num_nodes_with_color_or_null->push_back(num_nodes_left);
        }
      }
      const int64_t num_arcs = (directed ? 1 : 2) * num_expected_lines;
      graph.reset(new Graph(num_nodes, num_arcs));
      continue;
    }
    size_t space_pos = line.find(' ');
    int64_t node1 = -1;
    int64_t node2 = -1;
    bool parse_success = false;
    if (space_pos != std::string::npos) {
      if (absl::SimpleAtoi(absl::string_view(line.c_str(), space_pos),
                           &node1) &&
          absl::SimpleAtoi(absl::string_view(line.c_str() + space_pos + 1),
                           &node2)) {
        parse_success =
            node1 >= 0 && node1 < num_nodes && node2 >= 0 && node2 < num_nodes;
      }
    }
    if (!parse_success) {
      return absl::Status(
          absl::StatusCode::kInvalidArgument,
          absl::StrFormat(
              "In '%s', line %d: Expected two integers in the range [0, %d).",
              filename, num_lines_read, num_nodes));
    }
    // We don't add superfluous arcs to the graph, but we still keep reading
    // the file, to get better error messages: we want to know the actual
    // number of lines, and also want to check the validity of the superfluous
    // arcs (i.e. that their src/dst nodes are ok).
    if (num_lines_read > num_expected_lines + 1) continue;
    graph->AddArc(node1, node2);
    if (!directed && node1 != node2) graph->AddArc(node2, node1);
  }
  if (num_lines_read == 0) {
    return absl::Status(absl::StatusCode::kInvalidArgument,
                        "Unknown or empty file");
  }
  if (num_lines_read != num_expected_lines + 1) {
    return absl::Status(
        absl::StatusCode::kInvalidArgument,
        absl::StrFormat("The number of arcs/edges in '%s' (%d) does not match"
                        " the value announced in the header (%d)",
                        filename, num_lines_read - 1, num_expected_lines));
  }
  graph->Build();
  return graph.release();
}

template <class Graph>
absl::Status WriteGraphToFile(const Graph& graph, const std::string& filename,
                              bool directed,
                              const std::vector<int>& num_nodes_with_color) {
  FILE* f = fopen(filename.c_str(), "w");
  if (f == nullptr) {
    return absl::Status(absl::StatusCode::kInvalidArgument,
                        "Could not open file: '" + filename + "'");
  }
  // In undirected mode, we must count the self-arcs separately. All other arcs
  // should be duplicated.
  int num_self_arcs = 0;
  if (!directed) {
    for (const typename Graph::NodeIndex node : graph.AllNodes()) {
      for (const typename Graph::ArcIndex arc : graph.OutgoingArcs(node)) {
        if (graph.Head(arc) == node) ++num_self_arcs;
      }
    }
    if ((graph.num_arcs() - num_self_arcs) % 2 != 0) {
      fclose(f);
      return absl::Status(absl::StatusCode::kInvalidArgument,
                          "WriteGraphToFile() called with directed=false"
                          " and with a graph with an odd number of (non-self)"
                          " arcs!");
    }
  }
  absl::FPrintF(
      f, "%d %d", static_cast<int64_t>(graph.num_nodes()),
      static_cast<int64_t>(directed ? graph.num_arcs()
                                    : (graph.num_arcs() + num_self_arcs) / 2));
  if (!num_nodes_with_color.empty()) {
    if (std::accumulate(num_nodes_with_color.begin(),
                        num_nodes_with_color.end(), 0) != graph.num_nodes() ||
        *std::min_element(num_nodes_with_color.begin(),
                          num_nodes_with_color.end()) <= 0) {
      return absl::Status(absl::StatusCode::kInvalidArgument,
                          "WriteGraphToFile() called with invalid coloring.");
    }
    absl::FPrintF(f, " %d", num_nodes_with_color.size());
    for (int i = 0; i < num_nodes_with_color.size() - 1; ++i) {
      absl::FPrintF(f, " %d", static_cast<int64_t>(num_nodes_with_color[i]));
    }
  }
  absl::FPrintF(f, "\n");

  for (const typename Graph::NodeIndex node : graph.AllNodes()) {
    for (const typename Graph::ArcIndex arc : graph.OutgoingArcs(node)) {
      const typename Graph::NodeIndex head = graph.Head(arc);
      if (directed || head >= node) {
        absl::FPrintF(f, "%d %d\n", static_cast<int64_t>(node),
                      static_cast<uint64_t>(head));
      }
    }
  }
  if (fclose(f) != 0) {
    return absl::Status(absl::StatusCode::kInternal,
                        "Could not close file '" + filename + "'");
  }
  return ::absl::OkStatus();
}

}  // namespace util

#endif  // UTIL_GRAPH_IO_H_
