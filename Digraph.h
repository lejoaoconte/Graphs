#include <iostream>
#include <vector>
#include <list>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <stack>
#include <queue>


using namespace std;

struct Edge
{
    int v, w; // v -> w
    Edge(int v = -1, int w = -1) : v(v), w(w) {}
    
    int from() const { return v; }
    int to() const { return w; }
};

class Digraph
{
private:
    int V;
    int E;
    vector<list<Edge>> adj; // Lista de adjacência

public:
    Digraph(int V) : V(V), E(0)
    {
        if (V < 0)
            throw invalid_argument("O número de vértices não pode ser negativo");
        adj.resize(V);
    }

    Digraph(istream &in)
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
        adj[e.v].push_back(e);
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
                    edges.push_back(edge);
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
    
    void reverse()
    {
        vector<list<Edge>> revAdj(V);
        for (int v = 0; v < V; v++)
        {
            for (const Edge &e : adj[v])
            {
                revAdj[e.w].push_back(Edge(e.w, e.v));
            }
        }
        adj = revAdj;
    }   


    //Mostra o Grafo 
    void show(){
       vector<Edge> edges;
       cout << "Grafo com " << V << " vértices e " << E << " arestas." << endl;
       cout << "---------------------------------------------" << endl;
       for (int v = 0; v < V; ++v) {
           for (const auto& edge : adj[v]) {
                   cout << "  " << v << " -> " << edge.w << "\n";
           }
       }
        cout << "---------------------------------------------" << endl;
    }

    //Mostra o grafo no formato dot
    void showDot(){
       vector<Edge> edges;
       cout << "digraph {\n";
       cout << "  node [shape=circle];\n";
       for (int v = 0; v < V; ++v) {
           for (const auto& edge : adj[v]) {
                   cout << "  " << v << " -> " << edge.w << ";\n";
           }
       }
        cout << "}\n";
    }

    class adjIterator
    {
    private:
        const Digraph &G;
        int v;
        typename list<Edge>::const_iterator it;
    public:
        adjIterator(const Digraph &G, int v) : G(G), v(v) { it = G.adj[v].begin(); }
        int beg() { it = G.adj[v].begin(); return (it != G.adj[v].end()) ? it->w : -1; }
        int nxt() { if (it != G.adj[v].end()) ++it; return (it != G.adj[v].end()) ? it->w : -1; }
        bool end() const { return it == G.adj[v].end(); }
    };
};

