#include "EWGraph.h"
#include <map>

using namespace std;

class Process
{
private:
    EWGraph G;

public:
    Process(const EWGraph &G) : G(G) {}

    class MSTKruskal
    {
    private:
        vector<Edge> mst;
        double totalWeight;

    public:
        MSTKruskal(const EWGraph &G) : totalWeight(0.0)
        {
            UnionFind uf(G.getV());
            vector<Edge> edges = G.getAllEdges();
            sort(edges.begin(), edges.end());
            for (const Edge &e : edges)
            {
                int v = e.either();
                int w = e.other(v);
                if (!uf.connected(v, w))
                {
                    uf.unionSets(v, w);
                    mst.push_back(e);
                    totalWeight += e.weight;
                }
            }
        }
        vector<Edge> getMST() const { return mst; }
        double getTotalWeight() const { return totalWeight; }
    };

    class MSTPrim
    {
    private:
        vector<Edge> mst;
        double totalWeight;
        vector<Edge> edgeTo;
        vector<double> distTo;
        vector<bool> marked;
        IndexMinPQ<double> pq;

        void scan(EWGraph &G, int v)
        {
            marked[v] = true;
            for (const Edge &e : G.getAdj(v))
            {
                int w = e.other(v);
                if (marked[w])
                    continue;
                if (e.weight < distTo[w])
                {
                    distTo[w] = e.weight;
                    edgeTo[w] = e;
                    if (pq.contains(w))
                        pq.decreaseKey(w, distTo[w]);
                    else
                        pq.insert(w, distTo[w]);
                }
            }
        }
        void prim(EWGraph &G, int s)
        {
            distTo[s] = 0.0;
            pq.insert(s, 0.0);
            while (!pq.isEmpty())
            {
                int v = pq.delMin();
                if (edgeTo[v].either() != -1)
                {
                    mst.push_back(edgeTo[v]);
                    totalWeight += edgeTo[v].weight;
                }
                scan(G, v);
            }
        }

    public:
        MSTPrim(const EWGraph &G) : totalWeight(0.0), pq(G.getV())
        {
            for (int i = 0; i < G.getV(); i++)
            {
                distTo.push_back(numeric_limits<double>::infinity());
                edgeTo.push_back(Edge(-1, -1, 0.0));
                marked.push_back(false);
            }
            for (int i = 0; i < G.getV(); i++)
            {
                if (!marked[i])
                    prim(const_cast<EWGraph &>(G), i);
            }
        }
        vector<Edge> getMST() const { return mst; }
        double getTotalWeight() const { return totalWeight; }
    };

    class KClustering
    {
    private:
        UnionFind uf;
        double spacing;
        int k;
        int num_vertices;

    public:
        KClustering(const EWGraph &G, int k_clusters)
            : uf(G.getV()),
              spacing(0.0),
              k(k_clusters),
              num_vertices(G.getV())
        {
            if (k < 1 || k > num_vertices)
            {
                throw invalid_argument("O número de clusters (k) deve estar entre 1 e o número de vértices.");
            }

            vector<Edge> edges = G.getAllEdges();
            sort(edges.begin(), edges.end());

            int unions_performed = 0;
            int unions_needed = num_vertices - k;

            for (const Edge &e : edges)
            {
                if (unions_performed >= unions_needed)
                    break;

                int v = e.either();
                int w = e.other(v);

                if (!uf.connected(v, w))
                {
                    uf.unionSets(v, w);
                    unions_performed++;
                }
            }

            for (const Edge &e : edges)
            {
                int v = e.either();
                int w = e.other(v);
                if (!uf.connected(v, w))
                {
                    spacing = e.weight;
                    break;
                }
            }
        }

        double getSpacing() const
        {
            return spacing;
        }

        map<int, vector<int>> getClusters()
        {
            map<int, vector<int>> clusters;
            for (int i = 0; i < this->num_vertices; ++i)
            {
                int root = uf.find(i);
                clusters[root].push_back(i);
            }
            return clusters;
        }
    };
};

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Uso: " << argv[0] << " <arquivo_do_grafo>" << endl;
        return 1;
    }

    ifstream in(argv[1]);
    if (!in)
    {
        cerr << "Erro ao abrir o arquivo: " << argv[1] << endl;
        return 1;
    }

    // Cria o grafo a partir do arquivo
    EWGraph G(in);

    G.show();
    G.showDot();

    // Executa o algoritmo de Kruskal
    Process::MSTKruskal kruskal(G);
    cout << " ------- MST Kruskal -------" << endl;
    cout << "Peso total da MST (Kruskal): " << kruskal.getTotalWeight() << endl;
    cout << "Arestas na MST (Kruskal):" << endl;
    for (const Edge &e : kruskal.getMST())
    {
        cout << e.either() << " -- " << e.other(e.either()) << " (Peso: " << e.weight << ")" << endl;
    }

    // Executa o algoritmo de Prim
    Process::MSTPrim prim(G);
    cout << " ------- MST Prim -------" << endl;
    cout << "Peso total da MST (Prim): " << prim.getTotalWeight() << endl;
    cout << "Arestas na MST (Prim):" << endl;
    for (const Edge &e : prim.getMST())
    {
        cout << e.either() << " -- " << e.other(e.either()) << " (Peso: " << e.weight << ")" << endl;
    }

    // Executa o algoritmo de K-Clustering
    cout << "\n ------- K-Clustering -------" << endl;
    int k = 3;

    if (k > 0 && k <= G.getV())
    {
        Process::KClustering clustering(G, k);

        cout << "Executando para k = " << k << " clusters." << endl;
        cout << "Distância de separação (spacing): " << clustering.getSpacing() << endl;

        cout << "Clusters encontrados:" << endl;
        map<int, vector<int>> clusters = clustering.getClusters();

        int cluster_count = 1;
        for (const auto &pair : clusters)
        {
            cout << "  Cluster " << cluster_count++ << " (Raiz: " << pair.first << "): { ";
            for (int vertex : pair.second)
            {
                cout << vertex << " ";
            }
            cout << "}" << endl;
        }
    }
    else
    {
        cout << "Valor de k (" << k << ") inválido para um grafo com " << G.getV() << " vértices." << endl;
    }

    return 0;
}