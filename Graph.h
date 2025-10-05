#include <iostream>
#include <vector>
#include <list>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>

using namespace std;

/**
 * @struct Edge
 * @brief Representa uma aresta em um grafo não direcionado.
 */
struct Edge
{
    int v, w;

    Edge(int v = -1, int w = -1, double weight = 0.0) : v(v), w(w) {}

    int other(int vertex) const
    {
        if (vertex == v)
            return w;
        if (vertex == w)
            return v;
        throw invalid_argument("Vértice inválido na aresta");
    }
};

/**
 * @class Graph
 * @brief Representa um grafo não direcionado usando lista de adjacência.
 */
class Graph
{
private:
    int V;                  // Número de vértices
    int E;                  // Número de arestas
    vector<list<Edge>> adj; // Lista de adjacência

public:
    // Construtor a partir do número de vértices
    Graph(int V) : V(V), E(0)
    {
        if (V < 0)
            throw invalid_argument("O número de vértices não pode ser negativo");
        adj.resize(V);
    }

    // Construtor a partir de um arquivo de entrada
    Graph(istream &in)
    {
        if (!in)
            throw invalid_argument("Stream de entrada inválido.");
        in >> V;
        adj.resize(V);

        int totalEdges;
        in >> totalEdges;

        for (int i = 0; i < totalEdges; i++)
        {
            int v, w;
            in >> v >> w;
            addEdge(Edge(v, w));
        }
    }

    void addEdge(Edge e)
    {
        E++;
        int v = e.v;
        int w = e.w;
        adj[v].push_back(e);
        adj[w].push_back(Edge(w, v)); // Adiciona a aresta de volta para grafos não direcionados
    }

    int getV() const { return V; }
    int getE() const { return E; }

    // Retorna todas as arestas do grafo
    vector<Edge> getAllEdges() const
    {
        vector<Edge> edges;
        for (int v = 0; v < V; ++v)
        {
            for (const auto &edge : adj[v])
            {
                if (edge.v < edge.w)
                { // Adiciona cada aresta apenas uma vez
                    edges.push_back(edge);
                }
            }
        }
        return edges;
    }

    list<Edge> getAdj(int v) const
    {
        if (v < 0 || v >= V)
            throw invalid_argument("Vértice inválido");
        return adj[v];
    }           

    // Mostra o Grafo
    void show()
    {
        vector<Edge> edges;
        cout << "Grafo com " << V << " vértices e " << E << " arestas." << endl;
        cout << "---------------------------------------------" << endl;
        for (int v = 0; v < V; ++v)
        {
            for (const auto &edge : adj[v])
            {
                if (edge.v < edge.w)
                { // Adiciona cada aresta apenas uma vez
                    cout << "  " << v << " -- " << edge.w << "\n";
                }
            }
        }
        cout << "---------------------------------------------" << endl;
    }

    // Mostra o grafo no formato dot
    void showDot()
    {
        vector<Edge> edges;
        cout << "graph {\n";
        cout << "  node [shape=circle];\n";
        for (int v = 0; v < V; ++v)
        {
            for (const auto &edge : adj[v])
            {
                if (edge.v < edge.w)
                { // Adiciona cada aresta apenas uma vez
                    cout << "  " << v << " -- " << edge.w << ";\n";
                }
            }
        }
        cout << "}\n";
    }
};