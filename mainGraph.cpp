#include "Graph.h"

using namespace std;

class Process
{
private:
    Graph G;

public:
    Process(const Graph &G) : G(G) {}

    bool isComplete()
    {
        int V = G.getV(); // Pega o número de vértices do nosso grafo.
        int E = G.getE(); // Pega o número de arestas.

        // Retorna 'true' se o número de arestas for exatamente igual ao da fórmula, e 'false' caso contrário.
        return E == ((V * (V - 1)) / 2);
    }

    int degree(int v)
    {
        return G.getAdj(v).size();
    }

    bool isRegular()
    {
        // Se o grafo não tem vértices, por definição ele é regular (não tem como ser irregular).
        if (G.getV() == 0)
            return true;

        // Pega o grau do primeiro vértice (índice 0) como referência.
        int deg = degree(0);
        // Agora, vamos comparar o grau de todos os outros vértices com o do primeiro.
        for (int v = 1; v < G.getV(); v++)
        {
            // Se encontrarmos QUALQUER vértice com um grau diferente...
            if (deg != degree(v))
                return false; // ...o grafo não é regular, então já retornamos 'false'.
        }

        // Se o loop terminar e não encontrarmos nenhum grau diferente, o grafo é regular!
        return true;
    }

    bool isEulerian()
    {
        // Primeiro, a condição mais importante: o grafo precisa ser conexo.
        if (isConnected())
        {
            // Se for conexo, verificamos a segunda condição: o grau de cada vértice.
            for (int v = 0; v < G.getV(); v++)
            {
                // Se encontrarmos QUALQUER vértice com grau ímpar...
                if (degree(v) % 2 != 0)
                {
                    return false; // ...ele não pode ter um ciclo euleriano.
                }
            }
            // Se for conexo e todos os vértices tiverem grau par, então ele é euleriano.
            return true;
        }
        // Se não for conexo, já era.
        return false;
    }

    bool isEulerianPath()
    {
        // Novamente, precisa ser conexo.
        if (isConnected())
        {
            int oddDegree = 0; // Contador para os vértices de grau ímpar.
            for (int v = 0; v < G.getV(); v++)
            {
                // Se o grau do vértice for ímpar...
                if (degree(v) % 2 != 0)
                {
                    oddDegree++; // ...incrementa nosso contador.
                }
            }
            // Se o número de vértices com grau ímpar for 0 (é um ciclo euleriano) ou 2, então existe um caminho.
            if (oddDegree == 0 || oddDegree == 2)
                return true;
            else
                return false; // Se tiver 1, 3, 4, ... vértices de grau ímpar, não tem caminho.
        }
        return false;
    }

    bool isConnected()
    {
        // Grafos com 0 ou 1 vértice são considerados conexos.
        if (G.getV() <= 1)
        {
            return true;
        }

        // Usamos a nossa classe interna 'ConnectedComponents' que faz o trabalho sujo.
        ConnectedComponents cc(G);

        // Se o número de componentes for 1, ele é conexo.
        return cc.getCount() == 1;
    }

    bool isBipartite()
    {
        if (G.getV() == 0)
            return true; // Grafo vazio é bipartido.

        // Vetor de cores: 0=não visitado, 1=cor A, -1=cor B.
        vector<int> colors(G.getV(), 0);
        queue<int> q; // Usamos uma fila (BFS) para percorrer o grafo.

        // Esse 'for' é importante pra garantir que o algoritmo funcione mesmo se o grafo for desconexo.
        for (int i = 0; i < G.getV(); ++i)
        {
            // Se o vértice 'i' ainda não foi colorido, ele é o início de um novo componente.
            if (colors[i] == 0)
            {
                q.push(i);     // Adiciona na fila para começar a busca.
                colors[i] = 1; // Pinta com a primeira cor.

                // Inicia a busca em largura (BFS) a partir daqui.
                while (!q.empty())
                {
                    int u = q.front();
                    q.pop();

                    // Olha todos os vizinhos de 'u'.
                    for (const auto &edge : G.getAdj(u))
                    {
                        int v = edge.other(u);
                        // Se o vizinho 'v' ainda não foi colorido...
                        if (colors[v] == 0)
                        {
                            colors[v] = -colors[u]; // ...pinta com a cor OPOSTA à de 'u'.
                            q.push(v);              // E adiciona na fila pra visitar depois.
                        }
                        // CONFLITO! Se o vizinho 'v' já foi pintado e tem a MESMA COR que 'u'...
                        else if (colors[v] == colors[u])
                        {
                            return false; // ...significa que temos uma aresta ligando dois vértices da mesma cor. Não é bipartido.
                        }
                    }
                }
            }
        }
        // Se o código chegou até aqui, significa que conseguimos colorir o grafo todo sem conflitos.
        return true;
    }

    bool isPlanar()
    {
        int V = G.getV();
        int E = G.getE();

        // Grafos com poucos vértices são sempre planares.
        if (V < 3)
            return true;

        // Primeira condição (para grafos simples e conexos): E <= 3V - 6.
        // Se tiver mais arestas que isso, é impossível ser planar.
        if (E > 3 * V - 6)
            return false;

        // Segunda condição: se um grafo não tiver ciclos de tamanho 3 (triângulos),
        // a condição fica mais restrita: E <= 2V - 4.
        // O código abaixo tenta verificar se existe algum triângulo.
        bool hasTriangle = false;
        // Percorre cada vértice do grafo para checar se ele faz parte de um triângulo.
        for (int v = 0; v < V; ++v)
        {
            // Pega a lista de vizinhos do vértice 'v'.
            list<Edge> adjEdges = G.getAdj(v);
            vector<int> neighbors;
            for (const auto &edge : adjEdges)
            {
                neighbors.push_back(edge.other(v));
            }
            int n = neighbors.size();
            // Compara cada par de vizinhos (neighbors[i], neighbors[j])
            for (int i = 0; i < n; ++i)
            {
                for (int j = i + 1; j < n; ++j)
                {
                    // Verifica se esses dois vizinhos são conectados entre si.
                    for (const auto &edge : G.getAdj(neighbors[i]))
                    {
                        // Se sim, então v, neighbors[i] e neighbors[j] formam um triângulo.
                        if (edge.other(neighbors[i]) == neighbors[j])
                        {
                            hasTriangle = true;
                            break;
                        }
                    }
                    if (hasTriangle)
                        break;
                }
                if (hasTriangle)
                    break;
            }
            if (hasTriangle)
                break;
        }

        // Se não encontrou triângulos e o número de arestas viola a segunda condição...
        if (!hasTriangle && E > 2 * V - 4)
            return false; // ...então não é planar.

        // Se passou em todos os testes, a gente assume que é planar (lembrando que é uma simplificação).
        return true;
    }

    // Função para contar quantos componentes conexos existem no grafo
    int countConnectedComponents()
    {
        ConnectedComponents cc(G);
        return cc.getCount();
    }

    // Mostrar os componentes conexos
    void showConnectedComponents()
    {
        ConnectedComponents cc(G);
        int numComponents = cc.getCount();
        cout << "\n--- Componentes Conectados ---" << endl;
        cout << "Número de componentes conectados: " << numComponents << endl;

        for (int comp = 0; comp < numComponents; comp++)
        {
            cout << "\n>> Componente " << comp << ":" << endl;
            for (int v = 0; v < G.getV(); v++)
            {
                if (cc.getId(v) == comp)
                {
                    cout << "   Vértice: " << v << " | Arestas: ";
                    for (const Edge &edge : G.getAdj(v))
                    {
                        cout << edge.other(v) << " ";
                    }
                    cout << endl;
                }
            }
        }
        cout << "-------------------------------------" << endl;
    }

    // Mostrar o grau dos vértices
    void showDegrees()
    {
        cout << "\n--- Graus dos Vértices ---" << endl;
        for (int v = 0; v < G.getV(); v++)
        {
            cout << "Vértice " << v << " - Grau: " << degree(v) << endl;
        }
        cout << "-------------------------------------" << endl;
    }

    // Verificar quantos ciclos existem no grafo
    int countCycles()
    {
        int cycleCount = 0;
        vector<bool> visited(G.getV(), false);

        for (int i = 0; i < G.getV(); ++i)
        {
            if (!visited[i])
            {
                if (hasCycleDFS(i, -1, visited))
                {
                    cycleCount++;
                }
            }
        }

        return cycleCount;
    }

    // Função para detectar se o grafo contém ciclos
    bool isCyclic()
    {
        // Vetor pra marcar os vértices já visitados na busca atual.
        vector<bool> visited(G.getV(), false);

        // Loop principal para garantir que a gente teste todos os componentes do grafo.
        for (int i = 0; i < G.getV(); ++i)
        {
            if (!visited[i])
            {
                // Inicia a busca a partir de 'i', com um pai inválido (-1).
                if (hasCycleDFS(i, -1, visited))
                {
                    return true; // Se achou ciclo em qualquer componente, já pode parar.
                }
            }
        }

        // Se rodou tudo e não achou nenhum ciclo.
        return false;
    }

    // Algoritmo to find Minimum Spanning Tree (MST) using Kruskal's algorithm
    vector<Edge> kruskalMST()
    {
        vector<Edge> mst;
        vector<Edge> allEdges = G.getAllEdges();

        // Since edges don't have weights, we can use them in any order
        // For a proper MST with weights, we would sort edges by weight here

        // Union-Find data structure
        UnionFind uf(G.getV());

        // Process each edge
        for (const Edge &edge : allEdges)
        {
            int v = edge.v;
            int w = edge.w;

            // If vertices are not connected, add edge to MST
            if (!uf.connected(v, w))
            {
                uf.unionSets(v, w);
                mst.push_back(edge);

                // MST has exactly V-1 edges
                if (mst.size() == G.getV() - 1)
                    break;
            }
        }

        return mst;
    }

    void printMST()
    {
        if (!isConnected())
        {
            cout << "\n--- MST (Kruskal's Algorithm) ---" << endl;
            cout << "O grafo não é conexo. Não é possível encontrar uma MST." << endl;
            cout << "Mostrando árvores geradoras para cada componente:" << endl;

            ConnectedComponents cc(G);
            int numComponents = cc.getCount();

            for (int comp = 0; comp < numComponents; comp++)
            {
                cout << "\n>> Componente " << comp << ":" << endl;
                Graph componentGraph(G.getV());

                // Build subgraph for this component
                for (int v = 0; v < G.getV(); v++)
                {
                    if (cc.getId(v) == comp)
                    {
                        for (const Edge &edge : G.getAdj(v))
                        {
                            int w = edge.other(v);
                            if (cc.getId(w) == comp && v < w)
                            {
                                componentGraph.addEdge(Edge(v, w));
                            }
                        }
                    }
                }

                // Find MST for this component
                Process componentProcess(componentGraph);
                vector<Edge> componentMST = componentProcess.kruskalMST();

                cout << "   Arestas da árvore geradora:" << endl;
                for (const Edge &edge : componentMST)
                {
                    cout << "   " << edge.v << " -- " << edge.w << endl;
                }
            }
            return;
        }

        vector<Edge> mst = kruskalMST();

        cout << "\n--- MST (Kruskal's Algorithm) ---" << endl;
        cout << "Árvore Geradora Mínima com " << mst.size() << " arestas:" << endl;

        for (const Edge &edge : mst)
        {
            cout << "  " << edge.v << " -- " << edge.w << endl;
        }

        cout << "Total de arestas na MST: " << mst.size() << endl;
        cout << "-------------------------------------" << endl;
    }

private:
    // Union-Find data structure for Kruskal's algorithm
    class UnionFind
    {
    private:
        vector<int> parent;
        vector<int> rank;

    public:
        UnionFind(int n) : parent(n), rank(n, 0)
        {
            for (int i = 0; i < n; i++)
            {
                parent[i] = i;
            }
        }

        int find(int x)
        {
            if (parent[x] != x)
            {
                parent[x] = find(parent[x]); // Path compression
            }
            return parent[x];
        }

        void unionSets(int x, int y)
        {
            int rootX = find(x);
            int rootY = find(y);

            if (rootX != rootY)
            {
                // Union by rank
                if (rank[rootX] < rank[rootY])
                {
                    parent[rootX] = rootY;
                }
                else if (rank[rootX] > rank[rootY])
                {
                    parent[rootY] = rootX;
                }
                else
                {
                    parent[rootY] = rootX;
                    rank[rootX]++;
                }
            }
        }

        bool connected(int x, int y)
        {
            return find(x) == find(y);
        }
    };
    // Função auxiliar para detecção de ciclos usando DFS
    bool hasCycleDFS(int u, int parent, vector<bool> &visited)
    {
        visited[u] = true; // Marca o vértice atual como visitado.

        // Olha todos os vizinhos de 'u'.
        for (const auto &edge : G.getAdj(u))
        {
            int v = edge.other(u);
            // Se o vizinho 'v' ainda não foi visitado...
            if (!visited[v])
            {
                // ...continua a busca a partir dele. Se a chamada recursiva encontrar um ciclo...
                if (hasCycleDFS(v, u, visited))
                {
                    return true; // ...a gente só propaga o 'true' pra cima.
                }
            }
            // CICLO DETECTADO! Se o vizinho 'v' JÁ foi visitado E ele NÃO é o 'parent'
            // de onde acabamos de vir, significa que encontramos um caminho de volta (um ciclo).
            else if (v != parent)
            {
                return true;
            }
        }
        // Se exploramos todos os vizinhos de 'u' e não achamos ciclo, retorna false.
        return false;
    }

public:
    class depthFirstPaths
    {
    private:
        vector<bool> marked; // marcado[v] = existe um caminho de s para v?
        vector<int> edgeTo;  // edgeTo[v] = último vértice no caminho de s para v
        const Graph &G;      // referência ao grafo
        int source;          // vértice de origem

        void dfs(int v)
        {
            marked[v] = true;
            for (const Edge &e : G.getAdj(v)) // Itera sobre as arestas adjacentes
            {
                int w = e.other(v);
                if (!marked[w])
                {
                    edgeTo[w] = v;
                    dfs(w);
                }
            }
        }

    public:
        depthFirstPaths(const Graph &G, int s) : G(G), source(s)
        {
            marked.resize(G.getV(), false);
            edgeTo.resize(G.getV(), -1);
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
        const Graph &G;      // referência ao grafo
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

                for (const Edge &e : G.getAdj(v))
                {
                    int w = e.other(v);
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
        breadthFirstPaths(const Graph &G, int s) : G(G), source(s)
        {
            marked.resize(G.getV(), false);
            edgeTo.resize(G.getV(), -1);
            distTo.resize(G.getV(), 99999);
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

    class ConnectedComponents
    {
    private:
        vector<bool> marked; // marcado[v] = componente conectada de v?
        vector<int> id;      // id[v] = id da componente conectada de v
        int count;           // número de componentes conectadas
        const Graph &G;

        void dfs(int v, int componentId)
        {
            marked[v] = true;
            id[v] = componentId;
            for (const Edge &e : G.getAdj(v))
            {
                int w = e.other(v);
                if (!marked[w])
                {
                    dfs(w, componentId);
                }
            }
        }

    public:
        ConnectedComponents(const Graph &G) : G(G), count(0)
        {
            marked.resize(G.getV(), false);
            id.resize(G.getV(), -1);

            for (int v = 0; v < G.getV(); v++)
            {
                if (!marked[v])
                {
                    dfs(v, count);
                    count++;
                }
            }
        }

        int getCount() const { return count; }
        int getId(int v) const { return id[v]; }
    };
};

void deepFirstSearchAllComponents(const Graph &G)
{
    cout << "\n--- DFS em todos componentes ---" << endl;
    // Vetor global de visitados, para não iniciar uma nova busca em um vértice que já faz parte de um componente encontrado.
    vector<bool> visited(G.getV(), false);

    // Loop para passar por todos os vértices do grafo.
    for (int i = 0; i < G.getV(); ++i)
    {
        // Se o vértice 'i' ainda não foi visitado por NENHUMA busca anterior...
        if (!visited[i])
        {
            cout << "\n>> Componente Conectado (iniciando busca a partir do vértice " << i << "):" << endl;

            // ...inicia uma nova DFS a partir dele.
            Process::depthFirstPaths dfs(G, i);

            // Agora, vamos ver todos os vértices que são alcançáveis a partir de 'i'
            // e marcá-los como visitados no nosso vetor global.
            for (int v = 0; v < G.getV(); ++v)
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

/**
 * @brief Executa a BFS em todos os componentes de um grafo.
 */
void breadthFirstSearchAllComponents(const Graph &G)
{
    // A lógica é IDÊNTICA à da DFS, a única diferença é que usamos a classe 'breadthFirstPaths'.
    cout << "\n--- Executando Busca em Largura em Todos os Componentes ---" << endl;
    vector<bool> visited(G.getV(), false);

    for (int i = 0; i < G.getV(); ++i)
    {
        if (!visited[i])
        {
            cout << "\n>> Componente Conectado (iniciando busca a partir do vértice " << i << "):" << endl;

            // AQUI está a mudança: criamos um objeto BFS.
            Process::breadthFirstPaths bfs(G, i);

            for (int v = 0; v < G.getV(); ++v)
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
    Graph G(in);

    G.show();
    G.showDot();
    // Cria o objeto Process que executa as operações no grafo
    Process graph(G);

    cout << boolalpha;
    cout << "Euleriano: " << graph.isEulerian() << endl;
    cout << "Caminho Euleriano: " << graph.isEulerianPath() << endl;
    cout << "Conexo: " << graph.isConnected() << endl;
    cout << "Bipartido: " << graph.isBipartite() << endl;
    cout << "Planar (verificação simplificada): " << graph.isPlanar() << endl;
    cout << "Cíclico: " << graph.isCyclic() << endl;
    cout << "Completo: " << graph.isComplete() << endl;
    cout << "Regular: " << graph.isRegular() << endl;
    cout << "Número de Componentes Conectados: " << graph.countConnectedComponents() << endl;
    cout << "Número de Ciclos: " << graph.countCycles() << endl;

    // Mostra os componentes conectados e os graus dos vértices
    graph.showConnectedComponents();

    // Mostrar o grau dos vértices
    graph.showDegrees();

    // MST usando algoritmo de Kruskal
    graph.printMST();

    // Chama as funções globais para rodar DFS e BFS em todo o grafo.
    deepFirstSearchAllComponents(G);
    breadthFirstSearchAllComponents(G);

    return 0;
}