#include "Digraph.h"
#include <set>

using namespace std;

class Process
{
private:
    Digraph DG;

public:
    Process(const Digraph &G) : DG(G) {}

    int outDegree(int v)
    {
        return DG.getAdj(v).size();
    }

    int inDegree(int v)
    {
        int inDeg = 0;
        for (int i = 0; i < DG.getV(); i++)
        {
            for (const auto &edge : DG.getAdj(i))
            {
                if (edge.to() == v)
                {
                    inDeg++;
                }
            }
        }
        return inDeg;
    }

    int degree(int v)
    {
        return outDegree(v) + inDegree(v);
    }

    // Função que imprime o grau de ENTRADA (in-degree) de cada vértice.
    void countInDegree()
    {
        cout << "Grau de Entrada (In-degree) de cada vértice:" << endl;
        for (int v = 0; v < DG.getV(); v++)
        {
            cout << "Vértice " << v << ": " << inDegree(v) << " entradas" << endl;
        }
    }

    // Função que calcula e imprime o grau de SAÍDA (out-degree) de cada vertice.
    void countOutDegree()
    {
        cout << "Grau de Saída (Out-degree):" << endl;
        for (int v = 0; v < DG.getV(); v++)
        {
            cout << v << ": " << outDegree(v) << " saídas" << endl;
        }
    }

    // Função que imprime o grau (degree) de cada vértice.
    void countDegree()
    {
        cout << "Grau (Degree) de cada vértice:" << endl;
        for (int v = 0; v < DG.getV(); v++)
        {
            cout << "Vértice " << v << ": " << degree(v) << " conexões" << endl;
        }
    }

    void deepFirstSearchAllComponents()
    {
        cout << "\n--- DFS em todos componentes ---" << endl;
        // Vetor global de visitados, para não iniciar uma nova busca em um vértice que já faz parte de um componente encontrado.
        vector<bool> visited(DG.getV(), false);

        // Loop para passar por todos os vértices do grafo.
        for (int i = 0; i < DG.getV(); ++i)
        {
            // Se o vértice 'i' ainda não foi visitado por NENHUMA busca anterior...
            if (!visited[i])
            {
                cout << "\n>> Componente Conectado (iniciando busca a partir do vértice " << i << "):" << endl;

                // ...inicia uma nova DFS a partir dele.
                Process::depthFirstPaths dfs(DG, i);

                // Agora, vamos ver todos os vértices que são alcançáveis a partir de 'i'
                // e marcá-los como visitados no nosso vetor global.
                for (int v = 0; v < DG.getV(); ++v)
                {
                    if (dfs.hasPathTo(v))
                    {
                        visited[v] = true; // Marca no vetor global.

                        // Imprime o caminho encontrado.
                        stack<int> path = dfs.pathTo(v);
                        cout << "   Caminho de " << i << " para " << v << ": ";
                        while (!path.empty())
                        {
                            cout << path.top() << " ";
                            path.pop();
                        }
                        cout << endl;
                    }
                }
            }
        }
        cout << "-------------------------------------------------------------" << endl;
    }

    void breadthFirstSearchAllComponents()
    {
        // A lógica é IDÊNTICA à da DFS, a única diferença é que usamos a classe 'breadthFirstPaths'.
        cout << "\n--- Executando Busca em Largura em Todos os Componentes ---" << endl;
        vector<bool> visited(DG.getV(), false);

        for (int i = 0; i < DG.getV(); ++i)
        {
            if (!visited[i])
            {
                cout << "\n>> Componente Conectado (iniciando busca a partir do vértice " << i << "):" << endl;

                // AQUI está a mudança: criamos um objeto BFS.
                Process::breadthFirstPaths bfs(DG, i);

                for (int v = 0; v < DG.getV(); ++v)
                {
                    if (bfs.hasPathTo(v))
                    {
                        visited[v] = true;

                        stack<int> path = bfs.pathTo(v);
                        // A vantagem da BFS é que podemos pegar a distância do caminho mais curto.
                        int distance = bfs.distanceTo(v);

                        cout << "   Caminho (distância " << distance << ") de " << i << " para " << v << ": ";
                        while (!path.empty())
                        {
                            cout << path.top() << " ";
                            path.pop();
                        }
                        cout << endl;
                    }
                }
            }
        }
        cout << "-------------------------------------------------------------" << endl;
    }

    void showTopologicalOrder()
    {
        cout << "\n--- Ordem Topológica do Grafo ---" << endl;
        Process::topologicalSort topo(DG);
        stack<int> order = topo.getOrder();
        cout << "Ordem Topológica: ";
        while (!order.empty())
        {
            cout << order.top() << " ";
            order.pop();
        }
        cout << endl;
        cout << "-------------------------------------------------------------" << endl;
    }

    void showCycleDetection()
    {
        cout << "\n--- Detecção de Ciclos ---" << endl;
        Process::CycleDetection cycleDetector(DG);
        if (cycleDetector.hasCycle())
        {
            cout << "O grafo CONTÉM ciclos!" << endl;
            stack<int> cycle = cycleDetector.getCycle();
            cout << "Ciclo encontrado: ";
            while (!cycle.empty())
            {
                cout << cycle.top();
                cycle.pop();
                if (!cycle.empty())
                    cout << " -> ";
            }
            cout << endl;
        }
        else
        {
            cout << "O grafo NÃO contém ciclos (é um DAG - Directed Acyclic Graph)" << endl;
        }
        cout << "-------------------------------------------------------------" << endl;
    }

    void showCycleCounting()
    {
        cout << "\n--- Contagem de Ciclos ---" << endl;
        Process::CycleCounter cycleCounter(DG);
        cout << "Total de ciclos encontrados: " << cycleCounter.getCycleCount() << endl;
        cycleCounter.printAllCycles();
        cout << "-------------------------------------------------------------" << endl;
    }

    void showStronglyConnectedComponents()
    {
        cout << "\n--- Componentes Fortemente Conectadas (SCC) ---" << endl;
        Process::StronglyConnectedComponents scc(DG);
        cout << "Número de Componentes Fortemente Conectadas: " << scc.getCount() << endl;
        scc.printComponents();
        cout << "-------------------------------------------------------------" << endl;
    }

    // Verificar se o grafo contém ciclos.
    class CycleDetection
    {
    private:
        vector<bool> marked;  // vértice foi visitado?
        vector<bool> onStack; // vértice está na pilha de recursão atual?
        vector<int> edgeTo;   // aresta que levou a este vértice
        stack<int> cycle;     // ciclo encontrado (se existir)
        const Digraph &DG;    // referência ao grafo

        void dfs(int v)
        {
            onStack[v] = true;
            marked[v] = true;

            for (const Edge &e : DG.getAdj(v))
            {
                int w = e.to();

                if (hasCycle())
                    return; // Se já encontrou um ciclo, para

                if (!marked[w])
                {
                    edgeTo[w] = v;
                    dfs(w);
                }
                else if (onStack[w])
                {
                    // Encontrou uma aresta de retorno (back edge) - há um ciclo
                    for (int x = v; x != w; x = edgeTo[x])
                    {
                        cycle.push(x);
                    }
                    cycle.push(w);
                    cycle.push(v);
                }
            }
            onStack[v] = false;
        }

    public:
        CycleDetection(const Digraph &DG) : DG(DG)
        {
            marked.resize(DG.getV(), false);
            onStack.resize(DG.getV(), false);
            edgeTo.resize(DG.getV(), -1);

            for (int v = 0; v < DG.getV(); v++)
            {
                if (!marked[v] && !hasCycle())
                {
                    dfs(v);
                }
            }
        }

        bool hasCycle() const
        {
            return !cycle.empty();
        }

        stack<int> getCycle() const
        {
            return cycle;
        }
    };

    // Contar todos os ciclos no grafo
    class CycleCounter
    {
    private:
        vector<vector<int>> allCycles; // armazena todos os ciclos encontrados
        vector<bool> blocked;          // bloqueio para o algoritmo de Johnson
        vector<set<int>> blockedSet;   // conjunto de vértices bloqueados
        vector<int> stack_path;        // caminho atual sendo explorado
        const Digraph &DG;             // referência ao grafo
        int startVertex;               // vértice inicial para busca de ciclos

        bool dfs(int v, int start)
        {
            bool foundCycle = false;
            stack_path.push_back(v);
            blocked[v] = true;

            for (const Edge &e : DG.getAdj(v))
            {
                int w = e.to();

                if (w == start)
                {
                    // Encontrou um ciclo
                    vector<int> cycle = stack_path;
                    cycle.push_back(start); // Fecha o ciclo
                    allCycles.push_back(cycle);
                    foundCycle = true;
                }
                else if (!blocked[w])
                {
                    if (dfs(w, start))
                        foundCycle = true;
                }
            }

            if (foundCycle)
            {
                unblock(v);
            }
            else
            {
                for (const Edge &e : DG.getAdj(v))
                {
                    int w = e.to();
                    blockedSet[w].insert(v);
                }
            }

            stack_path.pop_back();
            return foundCycle;
        }

        void unblock(int v)
        {
            blocked[v] = false;
            for (int w : blockedSet[v])
            {
                if (blocked[w])
                    unblock(w);
            }
            blockedSet[v].clear();
        }

        // Versão simplificada que encontra ciclos básicos usando DFS
        void findAllCyclesDFS()
        {
            vector<bool> visited(DG.getV(), false);
            vector<bool> recStack(DG.getV(), false);
            vector<int> path;

            for (int v = 0; v < DG.getV(); v++)
            {
                if (!visited[v])
                {
                    dfsUtil(v, visited, recStack, path);
                }
            }
        }

        void dfsUtil(int v, vector<bool> &visited, vector<bool> &recStack, vector<int> &path)
        {
            visited[v] = true;
            recStack[v] = true;
            path.push_back(v);

            for (const Edge &e : DG.getAdj(v))
            {
                int w = e.to();

                if (recStack[w])
                {
                    // Encontrou um ciclo - extrair o ciclo do path
                    vector<int> cycle;
                    bool inCycle = false;
                    for (int vertex : path)
                    {
                        if (vertex == w)
                            inCycle = true;
                        if (inCycle)
                            cycle.push_back(vertex);
                    }
                    cycle.push_back(w); // Fecha o ciclo

                    // Verificar se é um ciclo novo (evitar duplicatas)
                    if (isNewCycle(cycle))
                    {
                        allCycles.push_back(cycle);
                    }
                }
                else if (!visited[w])
                {
                    dfsUtil(w, visited, recStack, path);
                }
            }

            path.pop_back();
            recStack[v] = false;
        }

        bool isNewCycle(const vector<int> &newCycle)
        {
            // Normalizar o ciclo (começar pelo menor vértice)
            vector<int> normalized = normalizeCycle(newCycle);

            for (const auto &existingCycle : allCycles)
            {
                vector<int> normalizedExisting = normalizeCycle(existingCycle);
                if (normalized == normalizedExisting)
                    return false;
            }
            return true;
        }

        vector<int> normalizeCycle(const vector<int> &cycle) const
        {
            if (cycle.empty())
                return cycle;

            // Encontrar o menor elemento
            auto minIt = min_element(cycle.begin(), cycle.end() - 1); // -1 porque último é repetição
            int minPos = distance(cycle.begin(), minIt);

            vector<int> normalized;
            // Começar do menor elemento
            for (int i = minPos; i < cycle.size() - 1; i++)
            {
                normalized.push_back(cycle[i]);
            }
            for (int i = 0; i < minPos; i++)
            {
                normalized.push_back(cycle[i]);
            }
            normalized.push_back(normalized[0]); // Fecha o ciclo

            return normalized;
        }

    public:
        CycleCounter(const Digraph &DG) : DG(DG)
        {
            blocked.resize(DG.getV(), false);
            blockedSet.resize(DG.getV());

            // Usar o método DFS simplificado
            findAllCyclesDFS();
        }

        int getCycleCount() const
        {
            return allCycles.size();
        }

        vector<vector<int>> getAllCycles() const
        {
            return allCycles;
        }

        void printAllCycles() const
        {
            for (int i = 0; i < allCycles.size(); i++)
            {
                cout << "Ciclo " << (i + 1) << ": ";
                for (int j = 0; j < allCycles[i].size(); j++)
                {
                    cout << allCycles[i][j];
                    if (j < allCycles[i].size() - 1)
                        cout << " -> ";
                }
                cout << endl;
            }
        }
    };

    class depthFirstPaths
    {
    private:
        vector<bool> marked; // marcado[v] = existe um caminho de s para v?
        vector<int> edgeTo;  // edgeTo[v] = último vértice no caminho de s para v
        const Digraph &DG;   // referência ao grafo
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
        const Digraph &DG;   // referência ao grafo
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
        const Digraph &DG;      // referência ao grafo
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

    // Implementar o algoritmo de Kosaraju-Sharir para mostrar os componentes fortes.
    class StronglyConnectedComponents
    {
    private:
        vector<bool> marked; // marcado[v] = vértice v foi visitado?
        vector<int> id;      // id[v] = id da componente fortemente conectada de v
        int count;           // número de componentes fortemente conectadas
        const Digraph &DG;
        Digraph reverseDG;              // Grafo transposto
        vector<vector<int>> components; // armazena os vértices de cada componente

        // DFS para obter ordem de finalização (primeira fase)
        void dfsFinishOrder(int v, vector<bool> &visited, stack<int> &finishOrder)
        {
            visited[v] = true;
            for (const Edge &e : DG.getAdj(v))
            {
                int w = e.to();
                if (!visited[w])
                {
                    dfsFinishOrder(w, visited, finishOrder);
                }
            }
            finishOrder.push(v); // Adiciona à pilha quando termina de processar
        }

        // DFS no grafo transposto (segunda fase)
        void dfsTransposed(int v, vector<int> &currentComponent)
        {
            marked[v] = true;
            id[v] = count;
            currentComponent.push_back(v);

            for (const Edge &e : reverseDG.getAdj(v))
            {
                int w = e.to();
                if (!marked[w])
                {
                    dfsTransposed(w, currentComponent);
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

            // FASE 1: DFS no grafo original para obter ordem de finalização
            vector<bool> visited(DG.getV(), false);
            stack<int> finishOrder;

            for (int v = 0; v < DG.getV(); v++)
            {
                if (!visited[v])
                {
                    dfsFinishOrder(v, visited, finishOrder);
                }
            }

            // FASE 2: DFS no grafo transposto na ordem reversa de finalização
            while (!finishOrder.empty())
            {
                int v = finishOrder.top();
                finishOrder.pop();

                if (!marked[v])
                {
                    vector<int> currentComponent;
                    dfsTransposed(v, currentComponent);
                    components.push_back(currentComponent);
                    count++;
                }
            }
        }

        int getCount() const { return count; }
        int getId(int v) const { return id[v]; }

        // Método adicional para obter os vértices de cada componente
        vector<vector<int>> getComponents() const { return components; }

        // Método para imprimir os componentes de forma organizada
        void printComponents() const
        {
            cout << "Componentes Fortemente Conectadas:" << endl;
            for (int i = 0; i < components.size(); i++)
            {
                cout << "Componente " << i << ": { ";
                for (int j = 0; j < components[i].size(); j++)
                {
                    cout << components[i][j];
                    if (j < components[i].size() - 1)
                        cout << ", ";
                }
                cout << " }" << endl;
            }
        }

        // Verificar se dois vértices estão fortemente conectados
        bool stronglyConnected(int v, int w) const
        {
            return id[v] == id[w];
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
    Digraph DG(in);
    DG.reverse(); // Inverte as arestas do grafo
    DG.show();
    DG.showDot();
    // Cria o objeto Process que executa as operações no grafo
    Process graph(DG);

    graph.showTopologicalOrder();

    graph.showCycleDetection();
    graph.showCycleCounting();

    graph.countInDegree();
    graph.countOutDegree();
    graph.countDegree();

    graph.deepFirstSearchAllComponents();

    graph.breadthFirstSearchAllComponents();

    graph.showStronglyConnectedComponents();

    return 0;
}