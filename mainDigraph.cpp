#include "Digraph.h"

using namespace std;

class Process
{
private:
    Digraph DG;

public:
    Process(const Digraph &G) : DG(G) {}

    int outDegree(int v){

    }

    int inDegree(int v){

    }

    int degree(int v){
        return outDegree(v)+inDegree(v);
    }

    class depthFirstPaths
    {
    private:
        vector<bool> marked; // marcado[v] = existe um caminho de s para v?
        vector<int> edgeTo;  // edgeTo[v] = último vértice no caminho de s para v
        const Digraph &DG;      // referência ao grafo
        int source;          // vértice de origem

        void dfs(int v)
        {
            marked[v] = true;
            for (const Edge &e : DG.getAdj(v)) // Itera sobre as arestas adjacentes
            {
                int w = e.to();
                if (!marked[w])
                {
                    edgeTo[w] = v;
                    dfs(w);
                }
            }
        }

    public:
        depthFirstPaths(const Digraph &DG, int s) : DG(DG), source(s)
        {
            marked.resize(DG.getV(), false);
            edgeTo.resize(DG.getV(), -1);
            dfs(s);
        }

        bool hasPathTo(int v) const
        {
            return marked[v];
        }

        stack<int> pathTo(int v) const
        {
            stack<int> path;
            if (!hasPathTo(v))
                return path; // Retorna pilha vazia

            for (int x = v; x != source; x = edgeTo[x])
            {
                path.push(x);
            }
            path.push(source); // Adiciona o vértice de origem
            return path;
        }
    };

    class breadthFirstPaths
    {
    private:
        vector<bool> marked; // marcado[v] = existe um caminho de s para v?
        vector<int> edgeTo;  // edgeTo[v] = último vértice no caminho de s para v
        vector<int> distTo;  // distTo[v] = distância de s para v
        const Digraph &DG;      // referência ao grafo
        int source;          // vértice de origem

        void bfs(int s)
        {
            queue<int> q;
            marked[s] = true;
            distTo[s] = 0;
            q.push(s);

            while (!q.empty())
            {
                int v = q.front();
                q.pop();

                for (const Edge &e : DG.getAdj(v))
                {
                    int w = e.to();
                    if (!marked[w])
                    {
                        edgeTo[w] = v;
                        distTo[w] = distTo[v] + 1;
                        marked[w] = true;
                        q.push(w);
                    }
                }
            }
        }

    public:
        breadthFirstPaths(const Digraph &DG, int s) : DG(DG), source(s)
        {
            marked.resize(DG.getV(), false);
            edgeTo.resize(DG.getV(), -1);
            distTo.resize(DG.getV(), 9999);
            bfs(s);
        }

        bool hasPathTo(int v) const
        {
            return marked[v];
        }

        int distanceTo(int v) const
        {
            return distTo[v];
        }

        stack<int> pathTo(int v) const
        {
            stack<int> path;
            if (!hasPathTo(v))
                return path; // Retorna pilha vazia

            for (int x = v; x != source; x = edgeTo[x])
            {
                path.push(x);
            }
            path.push(source); // Adiciona o vértice de origem
            return path;
        }
    };

    class topologicalSort
    {
    private:
        vector<bool> marked;    // marcado[v] = vértice v foi visitado
        stack<int> reversePost; // ordem topológica em ordem reversa
        const Digraph &DG;    // referência ao grafo        
        void dfs(int v)
        {
            marked[v] = true;
            for (const Edge &e : DG.getAdj(v))
            {
                int w = e.to();
                if (!marked[w])
                {
                    dfs(w);
                }
            }
            reversePost.push(v);
        }
    public:
        topologicalSort(const Digraph &DG) : DG(DG)
        {
            marked.resize(DG.getV(), false);            
            for (int v = 0; v < DG.getV(); v++)
            {
                if (!marked[v])
                {
                    dfs(v);
                }
            }
        }
        stack<int> getOrder() const { return reversePost; }
    };

    class StronglyConnectedComponents
    {
    private:
        vector<bool> marked; // marcado[v] = vértice v foi visitado?        
        vector<int> id;      // id[v] = id da componente fortemente conectada de v
        int count;           // número de componentes fortemente conectadas
        const Digraph &DG;
        Digraph reverseDG;   // Grafo transposto
        void dfs(int v)
        {
            marked[v] = true;
            id[v] = count;
            for (const Edge &e : reverseDG.getAdj(v))
            {
                int w = e.to();
                if (!marked[w])
                {
                    dfs(w);
                }
            }
        }
    public:
        StronglyConnectedComponents(const Digraph &DG) : DG(DG), reverseDG(DG)
        {
            reverseDG.reverse(); // Inverte o grafo
            marked.resize(DG.getV(), false);        
            id.resize(DG.getV(), -1);
            count = 0;
            for (int v = 0; v < DG.getV(); v++)
            {
                if (!marked[v])
                {
                    dfs(v);
                    count++;
                }
            }
        }   
        int getCount() const { return count; }
        int getId(int v) const { return id[v]; }

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
    Digraph DG(in);
    DG.reverse(); // Inverte as arestas do grafo
    DG.show();
    DG.showDot();
    // Cria o objeto Process que executa as operações no grafo
    Process graph(DG);

    Process::depthFirstPaths dfs(DG, 0); // Inicia a busca em profundidade a partir do vértice 0
    cout << " ------- DFS -------" << endl;
    for (int v = 0; v < DG.getV(); v++)
    {
        if (dfs.hasPathTo(v))
        {
            stack<int> path = dfs.pathTo(v);
            cout << "Caminho de 0 para " << v << ": ";
            while (!path.empty())
            {
                cout << path.top() << " ";
                path.pop();
            }
            cout << endl;
        }
        else
        {
            cout << "Não há caminho de 0 para " << v << endl;
        }
    }

    Process::breadthFirstPaths bfs(DG, 0); // Inicia a busca em largura a partir do vértice 0
    cout << " ------- BFS -------" << endl;

    for (int v = 0; v < DG.getV(); v++)
    {
        if (bfs.hasPathTo(v))
        {
            stack<int> path = bfs.pathTo(v);
            cout << "Caminho de 0 para " << v << ": ";
            while (!path.empty())
            {
                cout << path.top() << " ";
                path.pop();
            }
            cout << endl;
        }
        else
        {
            cout << "Não há caminho de 0 para " << v << endl;
        }
    }

    Process::topologicalSort topo(DG);
    stack<int> order = topo.getOrder();
    cout << " ------- Topological Sort -------" << endl;
    while (!order.empty())
    {
        cout << order.top() << " ";
        order.pop();
    }
    cout << endl;
    
    
    Process::StronglyConnectedComponents scc(DG);
    cout << "Número de componentes fortemente conectadas: " << scc.getCount() << endl;
    for (int v = 0; v < DG.getV(); v++)
    {
        cout << "Vértice " << v << " - componente " << scc.getId(v) << endl;
    } 

    for (int v = 0; v < DG.getV(); v++)
    {
        cout << "Vértice " << v << " - grau de entrada= " << graph.inDegree(v) 
         << " - grau de saída= " << graph.outDegree(v) 
         << "Vértice " << v << " - grau=  " << graph.degree(v) << endl;

    } 



    return 0;
}