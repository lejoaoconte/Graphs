#include <iostream>
#include <vector>
#include <list>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>     // Para std::iota
#include <algorithm>   // Para std::sort e std::swap

// Usar std:: para evitar conflitos de nomes
using namespace std;

// =================================================================
// ESTRUTURA DE DADOS AUXILIAR: Union-Find
// =================================================================
/**
 * @class UnionFind
 * @brief Implementa a estrutura de dados Union-Find (ou Disjoint Set Union)
 * com compressão de caminho e união por rank.
 * Essencial para o algoritmo de Kruskal.
 */
class UnionFind {
    private:
        vector<int> parent;
        vector<int> rank;
        int count;
    
    public:
        UnionFind(int n) : count(n) {
            parent.resize(n);
            rank.resize(n, 0);
            iota(parent.begin(), parent.end(), 0);
        }
    
        int find(int p) {
            while (p != parent[p]) {
                parent[p] = parent[parent[p]]; // Compressão de caminho
                p = parent[p];
            }
            return p;
        }
    
        void unionSets(int p, int q) {
            int rootP = find(p);
            int rootQ = find(q);
            if (rootP == rootQ) return;
    
            if (rank[rootP] < rank[rootQ]) {
                parent[rootP] = rootQ;
            } else if (rank[rootP] > rank[rootQ]) {
                parent[rootQ] = rootP;
            } else {
                parent[rootQ] = rootP;
                rank[rootP]++;
            }
            count--;
        }
    
        bool connected(int p, int q) {
            return find(p) == find(q);
        }
};

// =================================================================
// ESTRUTURA DE DADOS AUXILIAR: Fila de Prioridade Mínima Indexada
// =================================================================
/**
 * @class IndexMinPQ
 * @brief Implementa uma Fila de Prioridade Mínima Indexada.
 * Essencial para uma implementação eficiente do algoritmo de Prim e de Dijkstra.
 * @tparam Key O tipo da chave (prioridade), deve suportar comparação.
 */
template <typename Key>
class IndexMinPQ {
private:
    int maxN;
    int n;
    vector<int> pq;     // heap binário usando indexação baseada em 1
    vector<int> qp;     // inverso de pq: qp[pq[i]] = pq[qp[i]] = i
    vector<Key> keys;   // chaves[i] = prioridade de i

    void exch(int i, int j) { swap(pq[i], pq[j]); qp[pq[i]] = i; qp[pq[j]] = j; }
    bool greater(int i, int j) const { return keys[pq[i]] > keys[pq[j]]; }
    void swim(int k) { while (k > 1 && greater(k/2, k)) { exch(k, k/2); k = k/2; } }
    void sink(int k) {
        while (2*k <= n) {
            int j = 2*k;
            if (j < n && greater(j, j+1)) j++;
            if (!greater(k, j)) break;
            exch(k, j);
            k = j;
        }
    }

public:
    IndexMinPQ(int N) : maxN(N), n(0), pq(N + 1), qp(N + 1, -1), keys(N + 1) {}
    bool isEmpty() const { return n == 0; }
    bool contains(int i) const { return qp[i] != -1; }
    void insert(int i, Key key) { n++; qp[i] = n; pq[n] = i; keys[i] = key; swim(n); }
    int delMin() { int min = pq[1]; exch(1, n--); sink(1); qp[min] = -1; return min; }
    void decreaseKey(int i, Key key) { keys[i] = key; swim(qp[i]); }
};


/**
 * @struct Edge
 * @brief Representa uma aresta ponderada em um grafo.
 */
struct Edge
{
    int v, w;
    double weight;

    Edge(int v = -1, int w = -1, double weight = 0.0) : v(v), w(w), weight(weight) {}
    int either() const { return v; }
    int other(int vertex) const {
        if (vertex == v) return w;
        if (vertex == w) return v;
        throw invalid_argument("Vértice inválido na aresta");
    }
    bool operator<(const Edge& other) const { return weight < other.weight; }
};

/**
 * @class EWGraph
 * @brief Representa um grafo ponderado não direcionado.
 */
class EWGraph
{
private:
    int V;
    int E;
    vector<list<Edge> > adj;

public:
    EWGraph(int V) : V(V), E(0)
    {
        if (V < 0) throw invalid_argument("Número de vértices não pode ser negativo");
        adj.resize(V);
    }

    EWGraph(istream &in)
    {
        if (!in) throw invalid_argument("Stream de entrada inválido.");
        in >> V;
        adj.resize(V);
        int totalEdges;
        in >> totalEdges;
        for (int i = 0; i < totalEdges; i++) {
            int v, w;
            double weight;
            in >> v >> w >> weight;
            addEdge(Edge(v, w, weight));
        }
    }

    void addEdge(Edge e)
    {
        E++;
        int v = e.v;
        int w = e.w;
        adj[v].push_back(e);
        adj[w].push_back(Edge(w, v, e.weight));
    }

    int getV() const { return V; }
    int getE() const { return E; }

     // Retorna todas as arestas do grafo 
    vector<Edge> getAllEdges() const {
        vector<Edge> edges;
        for (int v = 0; v < V; ++v) {
            for (const auto& edge : adj[v]) {
                if (edge.v < edge.w) { // Adiciona cada aresta apenas uma vez
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

    //Mostra o Grafo 
     void show(){
        vector<Edge> edges;
        cout << "Grafo com " << V << " vértices e " << E << " arestas." << endl;
        cout << "---------------------------------------------" << endl;
        for (int v = 0; v < V; ++v) {
            for (const auto& edge : adj[v]) {
                if (edge.v < edge.w) { // Adiciona cada aresta apenas uma vez
                    cout << "  " << v << " -- " << edge.w << ": "<< edge.weight << "\n";
                }
            }
        }
         cout << "---------------------------------------------" << endl;
     }

     //Mostra o grafo no formato dot
     void showDot(){
        vector<Edge> edges;
        cout << "graph {\n";
        cout << "  node [shape=circle];\n";
        cout << "  edge [labeldistance=1.5];\n";
        for (int v = 0; v < V; ++v) {
            for (const auto& edge : adj[v]) {
                if (edge.v < edge.w) { // Adiciona cada aresta apenas uma vez
                    cout << "  " << v << " -- " << edge.w
                         << " [label=\"" << fixed << setprecision(2) << edge.weight << "\"];\n";
                }
            }
        }
         cout << "}\n";
     }

    class adjIterator
    {
    private:
        const EWGraph &G;
        int v;
        typename list<Edge>::const_iterator it;
    public:
        adjIterator(const EWGraph &G, int v) : G(G), v(v) { it = G.adj[v].begin(); }
        Edge beg() { it = G.adj[v].begin(); return (it != G.adj[v].end()) ? *it : Edge(-1, -1); }
        Edge nxt() { if (it != G.adj[v].end()) ++it; return (it != G.adj[v].end()) ? *it : Edge(-1, -1); }
        bool end() const { return it == G.adj[v].end(); }
    };
};

