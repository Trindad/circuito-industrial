/**
 * -------------------------------------------------------------------------------
 *  Algoritmo genético em linguagem C++ para a resolução do seguinte problema:
 *    Otimização de condutores elétricos em circuitos industriais.
 *                        ...
 *  Técnicas utilizadas:
 *      -Corte fixo para mutação e crossover
 *      -Algoritmo de Roulette Wheel
 *
-----------------------------------------------------------------------------------
%Copyright {2014} {Silvana Trindade}
%
%   Licensed under the Apache License, Version 2.0 (the "License");
%   you may not use this file except in compliance with the License.
%   You may obtain a copy of the License at
%
%       http://www.apache.org/licenses/LICENSE-2.0
%
%   Unless required by applicable law or agreed to in writing, software
%   distributed under the License is distributed on an "AS IS" BASIS,
%   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%   See the License for the specific language governing permissions and
%   limitations under the License.
*-----------------------------------------------------------------------------------
 */

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <random>
#include <stdio.h>
#include <climits>
#include <chrono>
#include <ctime>
#include "main.h"

#define BUFFER 9000
#define LIMIT 11 //número máximo de ligações que um nó pode ter
#define PONTO_DE_CORTE 3
#define MIN(a,b) (((a)<(b) && (a))?(a):(b))
#define MAX 100000
#define INF 9999


using namespace std;

int nIndividuos;
int tamanhoPopulacao;
int grauMaximo;

int grau[MAX];
int fila[MAX];

class Nodo{
public:

    int indice; //número do nó
    int grau; //número de ligações para o nó

    Nodo(){
        this->grau = 0;
    }

    void incrementa() {

        this->grau++;
    }

    int getGrau() {

        return this->grau;
    }
};

class Ligacao
{
public:
    Nodo origem;
    Nodo destino;
    double custo;

    Ligacao(Nodo _origem, Nodo _destino, double _custo){

        this->origem = _origem;
        this->destino = _destino;

        this->origem.incrementa();
        this->destino.incrementa();

        this->custo = _custo;
    }
 
};

class Circuito{
public:
    vector<Ligacao> circuito;   //armazena sequência do circuito ex: ABDACE
    double custo;               //armazena custo total do circuito
    double probabilidade;         //probabilidade do individuo ser selecionado
    
    Circuito(double _custo){
        this->custo = _custo;
    }
    
    Circuito(vector <Ligacao> _circuito){
        this->circuito = _circuito;
    }
    
    Circuito(){
        this->custo = -1;
    }
};

vector<Nodo> nodos;
vector<int> visited;

vector<int> ordenaCircuitos(vector<Circuito> circuitos, vector<int> indices) {

    vector<double> temporario;
    double aux; int aux1,i;

    for (i = 0; i < tamanhoPopulacao; i++)
    {
        temporario.push_back(circuitos[i].custo);
        indices.push_back(i);
    }

    for (i = 0; i < tamanhoPopulacao; i++) { 

        for (int j = 0; j <  tamanhoPopulacao; j++) { 

            if (temporario[i] < temporario[j])
            { 
                aux = temporario[i];
                aux1 = indices[i];

                temporario[i] = temporario[j]; 
                temporario[j] = aux; 

                indices[i] = indices[j];
                indices[j] = aux1;

            } 
        } 
    }
    return indices;
}

/**
 * Escreve circuito final no arquivo passado como parâmetro nomeArquivo
 */
void imprimeCircuitos(vector<Circuito> circuitos,string nomeArquivo){

    vector<char> alfabeto = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','U','V','W','X','Y','Z'};

    vector<int> indicesOrdenados;
    indicesOrdenados = ordenaCircuitos(circuitos,indicesOrdenados);
    ofstream saida;

    saida.open (nomeArquivo);

    if (nIndividuos <= (int)alfabeto.size())
    {
        for (int i = 0; i < (int)circuitos.size(); i++)
        {
            saida<<"------------Circuito "<<i+1<<"------------\n"<<endl;

            Circuito circuito = circuitos[indicesOrdenados[i]];

            for (int j = 0; j < (int)circuito.circuito.size(); j++)
            {
                int origem = circuito.circuito[j].origem.indice;
                int destino = circuito.circuito[j].destino.indice;

                saida<<" "<<alfabeto[origem]<<"\t"<<alfabeto[destino]<<"\t\n";
            }
            saida<<"custo total = "<<circuito.custo<<endl;
            saida << endl;
        }

        saida << "----------------------------------\n"<<endl;
    }
    else
    {
        for (int i = 0; i < (int)circuitos.size(); i++)
        {
            saida<<"------------Circuito "<<i+1<<"------------\n"<<endl;

            Circuito circuito = circuitos[indicesOrdenados[i]];

            for (int j = 0; j < (int)circuito.circuito.size(); j++)
            {
                saida<<" "<<circuito.circuito[j].origem.indice<<"\t"<<circuito.circuito[j].destino.indice<<"\t\n";
            }
            saida<<"custo total = "<<circuito.custo<<endl;
            saida << endl;
        }

        saida << "----------------------------------\n"<<endl;
    }

    saida.close();
}

/**
 * Imprime circuitos na tela
 */
void imprimeCircuitos(vector<Circuito> circuitos){

    for (int i = 0; i < (int)circuitos.size(); i++)
    {
        for (int j = 0; j < (int)circuitos[i].circuito.size(); j++)
        {
            cout<<circuitos[i].circuito[j].origem.indice<<"\t"<<circuitos[i].circuito[j].destino.indice<<"\t";
        }
        cout << endl;
    }
}

/**
 * Obtêm todos os custos dos cirtuitos
 */
double calculaCusto(vector<Circuito> circuitos) {

    double custoCircuito = 0;

    for (int i = 0; i < (int)circuitos.size(); i++)
    {
        custoCircuito = 0;

        for (int j = 0; j < (int)circuitos[i].circuito.size(); j++)
        {
            custoCircuito = custoCircuito + circuitos[i].custo+circuitos[j].custo;
        }
        circuitos[i].custo = custoCircuito;
    }
    return custoCircuito;
}

/**
 * Calcula a probalidade do circuito ser escolhido na roleta
 * Soma a aptidão de todos os individuos
 * Divide a aptidão do individuo x pela soma das aptidões
 * E retorna a probabilidade.
 */
double probabilidadeDeSelecao(vector<Circuito> circuitos, double custoCircuito ) {

    double custoTotal = 0;

    for (int i = 0; i < (int)circuitos.size(); i++)
    {
       custoTotal = custoTotal + circuitos[i].custo;
    }

    return (custoCircuito/custoTotal);
}

/**
 * Retorna o custo total de todos os circuitos
 */
double custoTotalDosCircuitos(vector<Circuito> circuitos) {

    double custoTotal = 0;

    for (int i = 0; i < nIndividuos; i++)
    {
        custoTotal = custoTotal + circuitos[i].custo;
    }

    return custoTotal;

}

/**
 * Gera um nó rândomico entre o valor minimo e máximo
 * não necessáriamente sempre um único pois um nó 
 * poderá ter grau maior que um.
 */
int randomNode(int minimum, int maximum) {

    random_device rd;
    mt19937_64 gen(rd());

    uniform_int_distribution<> dis(minimum, maximum);
    
    return  dis(gen);
}

vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


vector<string> split(const string &s, char delim) {
   vector<string> elems;
    split(s, delim, elems);
    return elems;
}

/**
 * Faz a leitura do arquivo, e armazena a matriz de adjacências do grafo
 */
vector<vector<double>> leArquivo(string nomeArquivo){

    vector<vector<double>> grafo;
    nIndividuos = 0;

    string line;
    

    ifstream arquivo (nomeArquivo);

    if (arquivo.good())
    {
        int lineCount = 0;

        while ( getline (arquivo,line) )
        {
            // cout << line << '\n';

            vector<string> distanciaNos = split(line,';'); //quebra string pelo caractere ';'
            
            /**
             * Escreve circuito 
             */
            vector<double> temp;

            for (int i = 0; i < (int)distanciaNos.size(); i++)
            {
                double distancia = atof(distanciaNos[i].c_str()); //converter string em double
                temp.push_back(distancia);
            }

            grafo.push_back(temp); //insere no final da fila 
            lineCount++;
        }
        arquivo.close();
        nIndividuos = lineCount;
    }
    else
    {
        cout << "ERRO: Não foi possível abrir o arquivo " << nomeArquivo << endl;
        exit(0);
    }


    return grafo;
}

/**
 * Gera um nodo que tenha grau menor que o limite
 */
int geraNodo(vector<vector<double>> grafo) {

    int origem = -1;
    grauMaximo = nIndividuos-1;

    int limite = grauMaximo < LIMIT ? grauMaximo : LIMIT;

    origem = randomNode(0,nIndividuos-1);

    /**
     * Verifica o grau da origem
     */
    if (nodos[origem].grau < limite)
    {   
        return origem;
    }
    return geraNodo(grafo);
}

/**
 * Gera um nodo destino que tenha grau menor que o limite
 */
int geraNodo(vector<vector<double>> grafo,vector<int> controlador,int origem, int contador) {

    if (contador == nIndividuos)
    {
        return -1;
    }

    grauMaximo = nIndividuos-1;

    int limite = grauMaximo < LIMIT ? grauMaximo : LIMIT;

    int destino = randomNode(0,nIndividuos-1);

    if (controlador[destino] >= 0)
    {
        controlador[destino] = -1;
        contador++;
    }
    
    /**
     * Verifica o grau da origem
     */
    if (nodos[destino].grau < limite && origem != destino && grafo[origem][destino] > 0)
    {   
        return destino;
    }


    return geraNodo(grafo,controlador,origem,contador);
}


/**
 * gera caminhos minimos
 */
Circuito geraCaminhoMinimo(vector<vector<double>> grafo, vector<Circuito> circuitos) {

    /**
     * Não pode o destino ser igual a origem
     * O grau do nó deve ser menor que 12
     * Deve haver uma ligação estabelecida pela matriz de entrada
     * Enquanto não haja ligação para um vértice no mínimo continue
     */

    Circuito circuito;
    int nNodes = 0;
    vector<int> temporario = vector<int> (nIndividuos,0);
    vector<vector<double>> grafoTemp = grafo;

    double custoTotal = 0,i = 0;

    int origem = 0;
    int destino = origem;

    while( nNodes < nIndividuos && i < (nIndividuos*11)) {

        /**
         * Inicia circuito pelo nó O|A
         */
        if (nNodes == 0)
        {
            origem = 0;
            temporario[origem] = 1;
            nNodes++;
        }
    
        destino = geraNodo(grafoTemp,temporario,origem,0); //gera um nó de destino diferente da origem

        if (destino == -1)
        {
            break;
        }

        if (temporario[destino] == 0)
        {
            temporario[destino] = 1;
            nNodes++;
        }
        else
        {
            continue;
        }

        nodos[origem].grau = nodos[origem].grau+1;
        nodos[destino].grau = nodos[destino].grau+1;

        grafoTemp[origem][destino] = -1;
        grafoTemp[destino][origem] = -1;

        
        custoTotal = custoTotal + grafo[origem][destino];

        circuito.circuito.push_back(Ligacao(nodos[origem],nodos[destino],grafo[origem][destino]));

        origem = destino;
        
        i++;
    }

    circuito.custo = custoTotal;

    return circuito;
}


/**
 * gera arvores
 */
Circuito geraCaminho(vector<vector<double>> grafo, vector<Circuito> circuitos) {

    /**
     * Não pode o destino ser igual a origem
     * O grau do nó deve ser menor que 12
     * Deve haver uma ligação estabelecida pela matriz de entrada
     * Enquanto não haja ligação para um vértice no mínimo continue
     */

    Circuito circuito;
    
    vector<int> temporario = vector<int> (nIndividuos,0);
    vector<int> aux = vector<int> (nIndividuos,0);
    vector<vector<double>> grafoTemp = grafo;

    double custoTotal = 0,i = 0;

     /**
     * Inicia circuito pelo nó O|A
     * destino recebe a origem
     */
    int origem = 0;
    int destino = origem;

    int nDestinos = 1;//inicia com 1 pois o vértice 0 nunca poderá ser destino
    int nNodes = 0;

    temporario[0] = 1;//inicializa o vetor para que 0 nunca seja destino

    // cout<<"destino "<<destino<<"\t "<<nIndividuos<<endl;
    for (int k = 0; k < nIndividuos; k++)
    {
        grafoTemp[k][destino] = -1;
    }

    /**
     * Um vértice poderá ser somente uma vez destino
     * Mas origem poderá ser até nIndividuos-1
     * Não pode ter ciclo no grafo
     * Grafo inicializa pelo vértice 0
     */
    while( nNodes < nIndividuos || nDestinos < nIndividuos) {

        destino = geraNodo(grafoTemp,temporario,origem,0); //gera um nó de destino diferente da origem

        /**
         * grafo esta com todas as posições com -1, ou seja,
         * todas as ligações possíveis já foram realizadas
         */
        if (destino == -1 || origem == -1 || destino == origem || nodos[origem].grau >= nIndividuos || nodos[destino].grau >= nIndividuos )
        {
            origem = geraNodo(grafoTemp);
            continue;
        }

        /**
         * Se o vértice já foi destino 
         * Origem igual ao Destino
         * Destino for o vértice inicial
         * Volta no inicio do laço
         */
        if(temporario[destino] == 1  || destino == 0)
        {
            continue;
        }

        /**
         * Se o vértice ainda não foi destino
         * Incrementa o número de vértices destinos
         * Todos devem ser destino ao menos uma vez 
         * Com exceção para zero (0)
         */
        if (temporario[destino] == 0)
        {
            temporario[destino] = 1;
            nDestinos++;
        }
        

        /**
         * Verifica se os nós já não foram inseridos
         * Caso não tiverem sido incrementa o contador
         * Fazendo a verificação tanto para origem quanto destino
         */
        if (aux[origem] == 0)
        {
            aux[origem] = 1;
            nNodes++;
        }

        if (aux[destino] == 0)
        {
            aux[destino] = 1;
            nNodes++;
        }

       
        nodos[origem].grau = nodos[origem].grau+1;
        nodos[destino].grau = nodos[destino].grau+1;

        
        /**
         * Insere -1 em todas as posições de destino 
         * Assim não será o vértice mais que 1 vez 
         */
        grafoTemp[destino][origem] = -1;
        grafoTemp[origem][destino] = -1;

        for (int k = 0; k < nIndividuos; k++)
        {
            grafoTemp[k][destino] = -1;
        }
        
        custoTotal = custoTotal + grafo[origem][destino];

        circuito.circuito.push_back(Ligacao(nodos[origem],nodos[destino],grafo[origem][destino]));

        origem = geraNodo(grafoTemp);//gera nova origem

        i++;//aumento o número de ligações do grafo
    }
    
    circuito.custo = custoTotal;

    return circuito;
}

/**
 * Verifica se existe caminho simples
 */
int pathR(vector<vector<int>> G, int v, int w,vector<int> visited)
{
    int t;

    if (v == w) 
    {
        return 1;
    }

    visited[v] = 1;

    for (t = 0; t < nIndividuos; t++) {

        if (G[v][t] == 1) {

            if (visited[t] == 0) 
            {
                if (pathR(G, t, w,visited))
                {
                    return 1;
                }
            }
        }
    } 
    return 0;
}

int DFS(vector<vector<int>> A,int v)
{
    visited[v] = 1;

    for(int i = 0 ;i < nIndividuos; i++) {

        if(visited[A[v][i]] == 0 && A[v][i])
        {
            DFS(A,i);
        }
    }
    return v;
}

bool profundidade(vector<vector<int>> A,int n)  
{  
    if(visited[n] == 1) return true;  

    visited[n] = 1;  

    for(int i = 0;i < nIndividuos; i++)  
    {  
        if(A[n][i] != 0)  
        {  
            if(profundidade(A,i)) return true;                       
        }  
    }  
    return false;    
}
/**
 * Retorna a cintura 
 */
int BFS(vector<vector<int>> A,int n, int origem) {

    int infinito = n+1;

    int i = 0,girth = infinito;
    int ini, fim;

    vector<int> distancia = vector<int>(n);

    /*
        Preenche vetor de distancia com um numero 'infinito'
    */
    for( i = 0; i < n; i++)
    {
        distancia[i] = infinito;
    }

    distancia[origem] = 0;

    ini = fim = 0;

    fila[fim++] = origem; 

    while(ini != fim) 
    {
        int no = fila[ini++];

        int grau_i = nodos[no].grau;

        for( i = 0; i < grau_i; i++) 
        {
            int viz = A[no][i];
            
            if(distancia[viz] == infinito) 
            {
                fila[fim++] = viz;
                distancia[viz] = distancia[no] + 1;
            }
            else
            {
                int temp = MIN(girth,distancia[no]+distancia[viz]+1);
                girth = temp >= 3 ? temp : girth;     
            }
        }
    }
    return girth;
}

/**
 * Verifica se existem todos os n individuos no circuito
 */
bool circuitoPossuiCiclo(Circuito circuito,vector<vector<int>> grafo) {

    /**
     * Se existir dois individuos não ocorre ciclo
     */
    if (nIndividuos < 3)
    {
        return false;
    }

    bool ehArvore = false;

    for (int i = 0; i < nIndividuos; i++)
    {
        ehArvore = profundidade(grafo,i);//verifica se existe ciclo
    }

    return ehArvore;
}

/**
 * Gera população inicial 
 */
vector<Circuito> geraPopulacaoInicial(vector<vector<double>> grafo, vector<Circuito> circuitos){

    for (int i = 0; i < tamanhoPopulacao;) {
    
        for (int j = 0; j < (int)nodos.size(); j++)
        {
          nodos[j].grau = 0;
        }

        Circuito circuito = geraCaminho(grafo,circuitos);

        vector<vector<int>> g = vector<vector<int>>(nIndividuos, vector<int>(nIndividuos, 0));
        visited = vector<int>(nIndividuos,0);

        for (int j = 0; j < (int)circuito.circuito.size(); j++)
        {
            int origem =  circuito.circuito[j].origem.indice;
            int destino = circuito.circuito[j].destino.indice;
            
            g[origem][destino] = 1;
            g[destino][origem] = 1;
        }

        if (circuitoPossuiCiclo(circuito,g) == false) 
        {
            continue;
        }

        int existe = 1;

        visited = vector<int>(nIndividuos,0);

        for (int j = 0; j < nIndividuos; j++)
        {
            existe = pathR(g,0,j,visited);

            if (existe == 0)
            {
                break;
            }
          
        }
      
        if (existe == 0)
        {
           continue;
        }

        circuitos.push_back(circuito);//gera um circuito randômico
   
        i++;
    }
    return circuitos;
}


bool avaliaResultado(vector<Circuito> circuitos){ //compara circuitos, sera utilizada pela atualizaPopulacao para avaliar o resultado
    
    int custoTotal = custoTotalDosCircuitos(circuitos);
    int n = circuitos.size();

    int mediaDosCustosCircuitos = floor(custoTotal/n); //média de custo dos circuitos

    /**
     * Desvio padrão de 5%
     */
    int baixo = mediaDosCustosCircuitos - floor( (mediaDosCustosCircuitos*10) / 100 ); //desvio padão de 5% para mais ou menos referente a média
    int alto = mediaDosCustosCircuitos + floor( (mediaDosCustosCircuitos*10) / 100 );
    int nCircuitos = 0;

    for (int i = 0; i < n; i++)
    {
        double custo_i = circuitos[i].custo;

        if (custo_i >= baixo && custo_i <= alto)
        {
            nCircuitos++;
        }
    }

    int dentroDaMedia = floor( (100*nCircuitos)/n ); //calcula quanto por cento da população esta dentro da média

    if (dentroDaMedia >= 85)
    {
        return true;//para a simulação
    }
    return false;
}

/**
 *Avalia se o filho tem todos os vértices no caminho
 */
bool possuiTodosOsVertices(Circuito circuito) {

    int temp[nIndividuos],i = 0, count = 0;

    for (int i = 0; i < nIndividuos; i++)
    {
        temp[i] = i;
    }

    i = 0;
   
    while(i < (int)circuito.circuito.size())
    {
       
        int origem = circuito.circuito[i].origem.indice;

        if (temp[origem] >= 0)
        {
            temp[origem] = -1;
            count++;
        }

        int destino = circuito.circuito[i].destino.indice;

        if (temp[destino] >= 0)
        {
            temp[destino] = -1;
            count++;
        }
        i++;
    }


    if (count == nIndividuos)
    {
        return true;
    }

    return false;
}

/**
 * Verifica se já não existe na população um caminho igual a um inidividuo
 */
bool vericaCaminhoIgual(Circuito pai,Circuito filho) {

    vector<int> destinos = vector<int>(nIndividuos,0);//controlador de quantas vezes um vértice foi destino

    if (pai.circuito.size() != filho.circuito.size())
    {
        return true;//pai é diferente do filho
    }
    int nIguais = 0;
    int n = pai.circuito.size();

    if (filho.circuito[0].origem.indice >= 1)
    {
        return true;//vértice não começa por zero
    }
    /**
     * Verifica quantos pares de individuos são identicos ao pai
     * Caso o caminho filho gerado for igual a um individuo da população 
     * Então não insere 
     */
    for (int i = 0; i < n; i++)
    {
        int origemPai = pai.circuito[i].origem.indice;
        int origemFilho = filho.circuito[i].origem.indice;

        int destinoPai = pai.circuito[i].destino.indice;
        int destinoFilho = filho.circuito[i].destino.indice;

        if (destinos[destinoFilho] == 1)
        {
            return true;
        }
        else
        {
            destinos[destinoFilho] = 1;
        }

        if (origemPai == origemFilho && destinoPai == destinoFilho)
        {
            nIguais++;
        }
    }
    
    if (nIguais == n)
    {
        return true;//caminhos iguais
    }

    return false;
} 

/**
 * Verifica ligações repetidas ex: u liga com v e v liga com u
 */
bool ligacaoRepetida(Circuito circuito, vector<vector<double>> grafo) {
    vector<vector<double>> grafoTemp = grafo;

    for (int i = 0; i < (int)circuito.circuito.size(); i++)
    {
        int origem = circuito.circuito[i].origem.indice;
        int destino = circuito.circuito[i].destino.indice;

        if (grafoTemp[origem][destino] <= 0)
        {
            return true;
        }

        grafoTemp[origem][destino] = 0;
        grafoTemp[destino][origem] = 0;
    }
    return false;
}

/**
 * Atualiza a população 
 */
vector<Circuito> atualizaPopulacao(vector<Circuito> circuitos,vector<Circuito> filhos,vector<vector<double>> grafo){ //retorna um novo vetor de ciruitos atualizado se necessário
    
    vector<int> indicesOrdenados;
    indicesOrdenados = ordenaCircuitos(circuitos,indicesOrdenados);
    int nFilhos = (int)filhos.size();

    while(nFilhos > 0)
    {
        for (int i = 0; i < (int)filhos.size(); i++)
        {
            for (int j = 0; j < (int)circuitos.size(); j++)
            {
                if (vericaCaminhoIgual(circuitos[j],filhos[i]) == true)
                {
                    filhos.erase(filhos.begin(),filhos.begin()+1);
                    break;
                }
                if (ligacaoRepetida(filhos[i],grafo) == true)
                {
                    filhos.erase(filhos.begin(),filhos.begin()+1);
                    break;
                }
            }
        }
        nFilhos--;
    }

    for (int i = (int)(circuitos.size()-1); i >= 0; i--)
    {
        int index = -1, j = 0;

        for (j = 0; j < (int)filhos.size(); j++)
        {

            bool possui = possuiTodosOsVertices(filhos[j]);

            if (possui == false)
            {
                continue;
            }

            double custo_pai = circuitos[i].custo;
            double custo_filho = filhos[j].custo;

            vector<vector<int>> grafoTemp =  vector<vector<int>> (nIndividuos,vector<int>(nIndividuos,0));

             /**
              * Monta grafo de um circuito
              */
            double custo_i = 0;

            for (int p = 0; p < (int)(filhos[j].circuito.size()); p++)
            {
                int origem = filhos[j].circuito[p].origem.indice;
                int destino = filhos[j].circuito[p].destino.indice;

                if (grafo[origem][destino] > 0)
                {
                    grafoTemp[origem][destino] = 1;
                    grafoTemp[destino][origem] = 1;
                    custo_i = (custo_i + (double)grafo[origem][destino]);
                }
            }
            
            filhos[j].custo = (double)custo_i;
            
            int todosPossuemFilhos = circuitoPossuiCiclo(filhos[j],grafoTemp);

            if (todosPossuemFilhos == false) 
            {
                continue;//vai para próximo filho
            }

            int existe = 1;
            vector<int> visited = vector<int>(nIndividuos,0);

            for (int k = 0; k < nIndividuos; k++)
            {
                existe = pathR(grafoTemp,0,k,visited);//verifica se existe um caminho de 0 a até k

                if (existe == 0)
                {
                   break;
                }  
            }

            if ( custo_pai > custo_filho && custo_filho != -1 && existe == 1)
            {
                index = j;
                break;
            }
        }

        if (index >= 0)
        {
            circuitos[i] = filhos[index];

            filhos[index].custo = 99999;
        }
        
    }
    return circuitos;
}

/**
 * são sorteados n pares de genes, e os elementos do par trocam de valor entre si
 */
vector<Circuito> mutacaoPorTroca(vector<vector<double>> grafo,vector<Circuito> circuito,int index,int nPares) {

    /**
     * Seleciona pares ligação randômica e faz a mutação
     * trocando os indivíduos posição (swap)
     */
    int origemUm = 0, destinoUm = 0,origemDois = 0, destinoDois = 0;
    int nPosicoes = circuito[index].circuito.size();

    int i = randomNode(0, (nPosicoes*2)-1 );
    int j = i,count = 0;
  
    while(count < nPares) {

        while(j == i) {

            j = randomNode(0, (nPosicoes*2)-1 );
        }

        int umPar = (int)(i/2);
        int doisPar = (int)(j/2);

        Circuito temp = circuito[index];

        int aux = 0;

        if (i%2)
        {
            if (j%2)
            {
                aux = temp.circuito[umPar].origem.indice;
                
                origemUm = temp.circuito[umPar].origem.indice = temp.circuito[doisPar].origem.indice; 
                origemDois = temp.circuito[doisPar].origem.indice = aux; 

                destinoUm = temp.circuito[umPar].destino.indice;
                destinoDois = temp.circuito[doisPar].destino.indice;
            }
            else
            {
                aux = temp.circuito[umPar].origem.indice;
                
                origemUm = temp.circuito[umPar].origem.indice = temp.circuito[doisPar].destino.indice; 
                destinoDois = temp.circuito[doisPar].destino.indice = aux; 

                origemDois = temp.circuito[doisPar].origem.indice;
                origemUm = temp.circuito[umPar].origem.indice;
            }
        }
        else
        {
            if (j%2)
            {
                
                aux = temp.circuito[umPar].destino.indice;
                
                destinoUm = temp.circuito[umPar].destino.indice = temp.circuito[doisPar].origem.indice ;
                origemDois = temp.circuito[doisPar].origem.indice = aux; 

                destinoDois = temp.circuito[doisPar].destino.indice;
                origemUm = temp.circuito[umPar].origem.indice;
            }
            else
            {
                aux = temp.circuito[umPar].destino.indice;

                destinoUm = temp.circuito[umPar].destino.indice = temp.circuito[doisPar].destino.indice;
                destinoDois = temp.circuito[doisPar].destino.indice = aux;

                origemDois = temp.circuito[doisPar].origem.indice;
                origemUm = temp.circuito[umPar].origem.indice; 
            }
        }

        /**
         * Substitui circuito e recalcula o custo
         */
        if (grafo[origemUm][destinoUm] > 0 && grafo[origemDois][destinoDois] > 0)
        {
            circuito[index] = temp;

            // Circuito aux = circuito[index];
            // circuito[index] = recalculaCusto(aux,grafo);
        }
        count++;
    }  

    return circuito;
}

/**
 * Verifica a existência de ligação entre um par de vértices
 */
int existeLigacao(vector<vector<double>> grafo,int origem, int destino) {

    if(grafo[origem][destino] > 0) 
    {
        return 1;
    }

    return 0;
}

/**
 *Soma ou subtrai 1 do indice do nó destino
 */
int mutacaoCreep(vector<vector<double>> grafo,int origem,int destino) {
   
    int novoDestino = destino,count = 1;

    while(true) {
        
        novoDestino = destino-count;

        if (novoDestino > 0)
        {
            if(grafo[origem][novoDestino] > 0) break;
        }
        else
        {
            novoDestino = destino+count;

            if (novoDestino < nIndividuos )
            {
               if (grafo[origem][novoDestino] > 0) break;
            }
        }

        if (count == nIndividuos)
        {
            break;
        }
        count++;
    }
    return novoDestino;
}

/**
 * Recalcula o valor do custo e realiza  a mutação caso seja necessário
 */
Circuito recalculaCusto(Circuito circuito,vector<vector<double>> grafo) {

    int custo = 0,origem = 0,destino = 0;

    /**
     * Recalcula o custo do circuito
     */
    for (int i = 0; i < (int)circuito.circuito.size(); i++)
    {
        origem = circuito.circuito[i].origem.indice;
        destino = circuito.circuito[i].destino.indice;

        if (grafo[origem][destino] <= 0)
        {
           return circuito;
        }

        custo = grafo[origem][destino]+custo;
    }
    circuito.custo = custo;

    return circuito;
}

/**
 * Fará a mutação caso o custo do filho for maior que um dos pais
 * Se a ligação entre eles é invalida
 * Então sorteia dentre os do grafo que ocorre ligação 
 * Faz a mutação e recalcula os custos
 */
vector<Circuito> verificaCasosDeMutacao(vector<vector<double>> grafo,vector<Circuito> circuitos,vector<int> indices,vector<Circuito> filhos,int taxaMutacao){

  
    
    int n = filhos.size();

    for (int i = 0; i < n; i++)
    {
        int tam = filhos[i].circuito.size();

        int nPares = (taxaMutacao*tam)/100;//obtêm o número de pares

        if (nPares < 1)
        {
            nPares = 1;
        }

        /**
         * Se não houver ligação entre um dos nós
         * Ou os nós destino e origem forem o mesmo
         * Seleciona o nó que tem ligação invalida
         * Substitui por um randômico que tem ligação valida
         * Depois recalcula o custo
         */
        Circuito temp = filhos[i];

        temp = recalculaCusto(temp,grafo);

        if (temp.custo >= filhos[i].custo)
        {
            /**
             * seleciona nPares de ligação para sofrer mutação
             */
            filhos = mutacaoPorTroca(grafo, filhos, i,nPares);  
        }
    }

    return filhos;
}   

vector<Circuito> crossover(vector<Circuito> circuitos,vector<int> indices) {

    vector<Circuito> filhos;

    int nFilhos = indices.size();

    for (int i = 0; i < nFilhos-1; i+=2)
    {
        Circuito paiUm = circuitos[indices[i]];
        Circuito paiDois = circuitos[indices[i+1]]; 

        int um = paiUm.circuito.size();
        int dois = paiDois.circuito.size();
        int count = 0;

        /**
         * Armazena os vértices para fazer crossover
         */
        for (int j = um-1, k = dois-1; count < PONTO_DE_CORTE; j--,k--)
        {
            Nodo temp = paiUm.circuito[j].destino;
            paiUm.circuito[j].destino =  paiDois.circuito[k].destino;
            paiDois.circuito[k].destino = temp;

            count++;

            if (count >= PONTO_DE_CORTE)
            {
                break;
            }

            temp = paiUm.circuito[j].origem;
            paiUm.circuito[j].origem =  paiDois.circuito[k].origem;
            paiDois.circuito[k].origem = temp;

            count++;
        }

        filhos.push_back(paiUm);
        filhos.push_back(paiDois);

        count = 0;

      
    }

    return filhos;
}

/**
 * Calcula a probabilidade de um circuito ser selecionado
 */
vector<Circuito> probabilidade(vector<Circuito> circuitos,double custoTotal,vector<int> indicesOrdenados) {

    double probabilidadeTotal = 0,custo_i,temp = 0;
    double fMax = circuitos[indicesOrdenados[tamanhoPopulacao-1]].custo,totalFR = 0;
    vector<double> f;

    for (int i = 0; i < tamanhoPopulacao; i++)
    {
        temp = indicesOrdenados[i];
        custo_i = circuitos[temp].custo;
        f.push_back(fMax-custo_i);

        totalFR = f[i]+totalFR;
    }
    for (int i = 0; i < tamanhoPopulacao; i++)
    {
        temp = indicesOrdenados[i];

        custo_i = circuitos[temp].custo;

        double fr = (fMax-custo_i)+1;
        double w = fr/totalFR;

        circuitos[temp].probabilidade = w;

        probabilidadeTotal = probabilidadeTotal + w;
    }

    return circuitos;
}

/**
 * Primeiramente a cada individuo é atribuido um grau de aptidão
 * em seguida rodasse a roleta, selecionando aleatoriamente um individuo
 * deverá retornar o vetor de circuitos selecionados
 */
vector<int> algoritmoRoleta(vector<Circuito> circuitos,vector<vector<double>> grafo,int N){ 

    int i = 0;
    
    vector<int> popSeleciona;//insere os indices da população selecionada para crossover

    vector<int> indicesOrdenados;
    indicesOrdenados = ordenaCircuitos(circuitos,indicesOrdenados);
    
    
    /**
     * Soma da aptidão/custo total dos individuos/circuitos
     * Em seguida calcula a proporção de cada individuo
     */
    double custoTotal = custoTotalDosCircuitos(circuitos);

    circuitos = probabilidade(circuitos,custoTotal,indicesOrdenados);

    /**
     * Selecionar até N individuos na população
     */
    for (i = 0; i < N;i++)
    {
        int r = randomNode(0,100); //valor aleatório entre 0 e N circuitos
       
        double number = ((double)r)/100;

        /**
         * Percorrer sequencialmente os individuos da população
         * Acumulando em S o valor da aptidão dos já percorridos
         * Se for maior ou igual a r seleciona o individuo
         */
        double S = 0;
        

        for (int j = 0; j < tamanhoPopulacao; j++)
        {
            int existe = 0;

            S = S + circuitos[indicesOrdenados[j]].probabilidade;

            if (S >= number)
            {
                for (int k = 0; k < (int)popSeleciona.size(); k++)
                {
                    if (popSeleciona[k] == indicesOrdenados[j])
                    {
                        existe++;
                        break;
                    }
                }
                
                if (existe == 0)
                {
                    popSeleciona.push_back(indicesOrdenados[j]); //insere o indice do individuo 
                    break;
                }
                else
                {
                    i--;
                    break;
                }
            } 
            
        }
    }  
    return popSeleciona; 
}

int main(int argc, const char * argv[])
{
    if(argc <= 1)
    {
        cout<<"Nenhum arquivo especificado"<<endl;
        return 0;
    }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
    cout<<"\n******************************************************************"<<endl;
    cout << "| Algoritmo Genético para a Otimização de Circuitos Industriais! |\n";
    cout<<"******************************************************************\n"<<endl;
   
    int N = 0;

    cout << "Insira o tamanho da população inicial: " << endl;
    cin >> tamanhoPopulacao;

    if (tamanhoPopulacao < 2)
    {
        tamanhoPopulacao = 2;//permite com que haja crossover
    }

    while(true) {
       
        cout<< "Qual a taxa de crossover (\%):"<<endl;
        cin>>N;

        N = (int)(floor( (tamanhoPopulacao*N)/100 ));

        if (N >= 2 && N <= tamanhoPopulacao)
        {
            if (N%2)
            {
                N--;//caso pegue número impar de caminhos
            }
            break;
        }
    }

    int taxaMutacao = 1;//garante que ao menos um par será trocado

    cout << "Qual a taxa de mutação (\%):"<< endl;
    cin >> taxaMutacao;

    vector<vector<double>> grafo = leArquivo(argv[1]); //obtêm a matriz de ligações
    
    nodos = vector<Nodo>(nIndividuos); //para manter o controle do grau dos nós

    /**
     * Inicializa a população
     */
    for (int i = 0; i < nIndividuos; i++)
    {
        nodos[i].indice = i;
        nodos[i].grau = 0;
    }
    
    vector<Circuito> circuitos;

    circuitos = geraPopulacaoInicial(grafo,circuitos);

    int nIteracoes = 0;

    while(true) {

        vector<int> indices = algoritmoRoleta(circuitos,grafo,N); //retorna os indices dos circuitos selecionados


        vector<Circuito> filhos = crossover(circuitos,indices); //retorna os filhos gerados a partir do crossover
        
        if (filhos.size() == 0) continue;
        /**
         * retorna os filhos que sofrerão ou não mutação
         * caso o novo circuito tenha todos os vértices
         */

        filhos = verificaCasosDeMutacao(grafo,circuitos,indices,filhos,taxaMutacao); 
     
        circuitos = atualizaPopulacao(circuitos,filhos,grafo);

        bool avalia = avaliaResultado(circuitos);

        if (avalia == true || nIteracoes == 4)
        {
            break;
        }

        nIteracoes++;
    }

    
    end = std::chrono::system_clock::now();
 
    time_t end_time = chrono::system_clock::to_time_t(end);

    string nomeArquivo = "circuits_";
    string data = ctime(&end_time);//obtêm a data 
    data = data.substr(0, data.size()-1);//elimina ultimo caractere lixo
    string extencao = ".txt";
    nomeArquivo = nomeArquivo+data+extencao;

    imprimeCircuitos(circuitos,nomeArquivo);

    return 0;
}
