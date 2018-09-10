/*Function description
*
*
*/
/*
1) definir espacio de estados y controles ($A_{convex}$ y $\Omega_{estados}$)
2) para todos los nodos xi en $\Omega_{estados}$ hacer: (funcion optimal solution)
  2.1) Pararse en el nodo y determinar xi_{i\delta} = xi + h*f(xi,a), para todo a \in A_xi={delta_{xi} \in $\Omega_{estados}}$
  2.3) ver entre cuales nodos estoy con x_{i\delta} calcular $\lambda$
  2.4) Resolver $(T(u))_{i} = min_{a \in A:xi}(1-lamb*h)$\lambda$u + hF(a))_{i}$
3) determinar los $\alpha$ (feedback control syntesis)

*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
//#include <algorithm>
/*

daton  -> define los vértices inicial y final del intervalo para cada variable de estado  (2^n_x)
daton  -> define los vértices inicial y final del intervalo para cada variable de control (2^n_c)
x      -> Cada intervalo particionado para los estados.
a      -> Cada intervalo particionado para los controles.
nodo   -> (dn x ln) este es el mesh, o el espacio particionado de las variables de estado.
ln     -> Cantidad de nodos.
lc     -> Cantidad de controles
u_sol  -> minimo sobre todos los posibles controles de un funcional de costo dado en la funcion ele
*/

double **daton, **datoc, **x, **a, **nodo, **control;//u_sol era variable global por los threads
int *l,*lc, *lp;
//double **nodos_test,*x_piv;

struct posiciones {

   int fi, fs, dimn, dimc;
   double hh;
};

typedef struct adm_control{
    int val;
    double *lambda;
    int *ind;
    struct adm_control * next;
} adm_control_t;

typedef struct nodes_inf{
    int n;
    struct nodes_inf * next_n;
    //adm_control_t * next_inf;
    struct adm_control *next_inf;
}nodes_inf_t;

typedef struct T_cal{
    double value;
    struct T_cal * nex;
}T_cal_t;

void print_list_controls(adm_control_t * head, int dn) {
    int aux, i, j;
    adm_control_t * current = head;
 //this is just for 3D
    if(current->next != NULL){/*
            printf("Node\n");
            for(i = 0; i < dn; i++){
                printf(" %f, ", nodo[current->node][i]);
            }printf("\n");*/
    printf("list of admissible controls and lambdas\n");
    while (current != NULL) {
         printf("admissible control\n");
         for(i = 0; i < dn; i++){
                printf(" %f ", control[current->val][i]);
         }printf("\n");
         printf("Associated convex coefficients\n");
         for(i = 0; i < dn + 1; i ++){
                printf(" lamb[%d] %f \n", i, current->lambda[i]);
                if(current->lambda[i] < -0.0000001){
                        printf("Negative lambda");
                    while(1);
                }
                printf("node[%d]\n", current->ind[i]);
                for(j = 0; j < dn; j++){
                       printf(" %f, ", nodo[current->ind[i]][j]);
                }printf("\n");
        }printf("\n");
        current = current->next;
    }
    }
    else{
        printf("no admissible controls\n");
    }
}


void  print_list_test(nodes_inf_t * body, int dn){

     int aux, i, j;
    nodes_inf_t * current = body;

    while(current != NULL){
            printf("Node\n");
              for(i = 0; i < dn; i++){
                printf(" %f,", nodo[current->n][i]);
              }printf("\n");
            if(current->next_inf->next != NULL){
               printf("list of admissible controls and lambdas\n");
              adm_control_t * current1 = current->next_inf;

               while (current1 != NULL) {
                  printf("admissible control\n");
                  for(i = 0; i < dn; i++){
                     printf(" %f, ", control[current1->val][i]);
                  }printf("\n");
                  printf("Associated convex coefficients\n");
                  for(i = 0; i < dn + 1; i ++){
                     printf(" lamb[%d] %f \n", i, current1->lambda[i]);
                     printf("node[%d]\n", current1->ind[i]);
                     for(j = 0; j < dn; j++){
                        printf(" %f, ", nodo[current1->ind[i]][j]);
                     }printf("\n");
                  }printf("\n");
                  current1 = current1->next;
               }
            }
            else{
                 printf("no admissible controls\n");
            }
            current = current->next_n;
    }
}


void  print_list_nodes_and_controls(nodes_inf_t * body, int dn){

     int aux, i, j;
    nodes_inf_t * current = body;
    // adm_control_t * current1 = body->next_inf;

    while(current != NULL){
            printf("Node\n");
              for(i = 0; i < dn; i++){
                printf(" %f,", nodo[current->n][i]);
              }printf("\n");
            if(current->next_inf->next != NULL){
               printf("list of admissible controls and lambdas\n");
               while (current->next_inf != NULL) {
                  printf("admissible control\n");
                  for(i = 0; i < dn; i++){
                     printf(" %f, ", control[current->next_inf->val][i]);
                  }printf("\n");
                  printf("Associated convex coefficients\n");
                  for(i = 0; i < dn + 1; i ++){
                     printf(" lamb[%d] %f \n", i, current->next_inf->lambda[i]);
                     printf("node[%d]\n", current->next_inf->ind[i]);
                     for(j = 0; j < dn; j++){
                        printf(" %f, ", nodo[current->next_inf->ind[i]][j]);
                     }printf("\n");
                  }printf("\n");
                  current->next_inf = current->next_inf->next;
               }
            }
            else{
                 printf("no admissible controls\n");
            }
            current = current->next_n;
    }
}

void push(adm_control_t * head, int val, double *lamb, int dn, int *index) {
    int i;
    adm_control_t * current = head;
    while (current->next != NULL) {
        current = current->next;
    }
    current->next = calloc(1,sizeof(adm_control_t));
    current->next->val = val;
     //current->next->nod = node;
    current->next->lambda = calloc(dn + 1,sizeof(double));
    current->next->ind = calloc(dn + 1,sizeof(int));

    for(i = 0; i < dn + 1; i ++){
        current->next->lambda[i] = lamb[i];
    }
    for(i = 1; i < dn + 1; i ++){
        current->next->ind[i-1] = index[i];
    }
    current->next->ind[dn] = index[0];
    current->next->next = NULL;
}

void push_node(nodes_inf_t * head, int number){
nodes_inf_t * current = head;

 while (current->next_n != NULL) {
        current = current->next_n;
    }
    current->next_n = calloc(1,sizeof(nodes_inf_t));
    //current->next_inf = calloc(1,sizeof(adm_control_t));
    current->next_n->n = number;
    //current->next_inf = NULL;
    current->next_n->next_n = NULL;
   // printf("number node: %d\n", current->next_n->n);

}

 void push_nodes_controls(adm_control_t * head, nodes_inf_t * body, int dn){
 int aux, i, j;

 adm_control_t * current = head;
 nodes_inf_t * actual = body;

 while(actual->next_n != NULL){
    actual = actual->next_n;
 }
// actual -> next_inf = (adm_control_t *)malloc(sizeof(adm_control_t *));
 actual -> next_inf = head;
 }

/*create(int j, int m, int *l, int d1, int *p1, int *p)
Genera la malla de nodos.
j  -> inicial 0, indica en que dimensión está, si está en la última, guarda la información
d1 -> indica la dimensión del espacio de estados
m  -> inicial 0, indice de iteracion en las matrices
l  -> cantidad de nodos
p1 -> vector de maximos de dimension
p -> arreglo de dim d1 que itera en hasta la máxima dimension
 */

int* create(int j, int m, int dn, int *pn, int *pd){

  int i, aux1, aux2;

  if(j == (dn)){
    for(i = 0; i < dn; i++){
      aux1 = pd[i];
      aux2 = l[0];
      nodo[aux2][i] = x[i][aux1];
    }
    l[0] = l[0] + 1;
  }
  else{
    for(pd[m] = 0; pd[m] < pn[m]; pd[m]++){
      create(j+1, m+1, dn, pn, pd);
    }
  }
}

int* create_c(int j, int m, int dn, int *pn, int *pd){
  int i, aux1, aux2;
  if(j == (dn)){
    for(i = 0; i < dn; i++){
      aux1 = pd[i];
      aux2 = lc[0];
      control[aux2][i] = a[i][aux1];
    }
    lc[0] = lc[0] + 1;
  }
  else{
    for(pd[m] = 0; pd[m] < pn[m]; pd[m]++){
      create_c(j+1, m+1, dn, pn, pd);
    }
  }
}

int f(double *y_out, int ln, int lc,double h,int dc,int dn)
{
  int i, j;
  if (dn == 3){
    for (i = 0; i < dc; i++){
      y_out[i]=h*control[lc][i]+nodo[ln][i];
    }
  }
  if(dn == 2){
  double M1[2][2], M2[2], y_out_aux[2], yout_aux2[2], sum;

  M1[0][0] = 0;
  M1[0][1] = 1;
  M1[1][0] = -1;
  M1[1][1] = 0;

  M2[0] = 0;
  M2[1] = 1;

  for(i = 0; i < dn; i++){
    y_out_aux[i] = M2[i] * nodo[ln][1] * control[lc][0];
  }
  for(i = 0; i <dn; i++){
          sum = 0;
    for(j = 0; j< dn; j++){
       sum = M1[i][j]* nodo[ln][i] + sum;
    }
    yout_aux2[i] = sum;
  }
  for(i = 0; i < dn; i++){
    y_out[i] =  h*(y_out_aux[i] + yout_aux2[i]) + nodo[ln][i];
  }
  }

}

int in_region(double *y_out, int dn, int *p){

  int k, j, cont, aux;

  cont = 0;
  for(k = 0; k < dn; k++){
    for(j = 0; j < p[k]-1; j++){
      if((x[k][j] <= y_out[k])){
	   if((y_out[k] < x[k][j+1])){
	    cont = cont + 1;
	    j = p[k]-1;
	   }
      }
	   if(j == p[k]-2){
        if((y_out[k]== x[k][j+1])){
            cont = cont + 1;
	        j = p[k];
        }
	   }

    }
  }
  if(cont == dn){
    aux = 1;
    return aux;
  }
  else{
    aux = 0;
    return aux;
  }
}

/*
  Function index_func: return the index node of the possible neghbour node given in possible_nodes
  possible_nodes dn
  aux2 \in {0,....,pow(2,dn)}
  index_nodes pow(2,dn)
*/
int *index_func(double *possible_nodes, int aux2, int *index_nodes, int dn){

int j, i, cont;
double epsilon;

epsilon = 0.00001;
for(j = 0; j < l[0]; j++){
     cont = 0;
     for(i = 0; i < dn; i++){
         if(possible_nodes[i] > nodo[j][i] - epsilon && possible_nodes[i] < nodo[j][i] + epsilon){
            cont = cont + 1;
         }
     }
     if(cont == dn){
      index_nodes[aux2] = j;
     }
}
}

int* create_pos(int j, int m, int dn, int *pnp, int *pdp, double **possible, double *possible_nodes, int *index_nodes){
  int i, aux1, aux2;

  if(j == (dn)){
    for(i = 0; i < dn; i++){
      aux1 = pdp[i];
      aux2 = lp[0];
      //possible_nodes[aux2][i] = possible[i][aux1];
      possible_nodes[i] = possible[i][aux1];
    }

	index_func(possible_nodes, aux2, index_nodes, dn); //*entrega el indice del posible nodo
    lp[0] = lp[0] + 1;
  }
  else{
    for(pdp[m] = 0; pdp[m] < pnp[m]; pdp[m]++){
      create_pos(j+1, m+1, dn, pnp, pdp, possible, possible_nodes, index_nodes);
    }
  }
  //find(possible_nodes, possible_nodes);
}

double *neighbourhood(double **possible, double  *possible_nodes, double *y_out, int dn, int *pn, int *pnp, int *pd, int *pdp, int *index_nodes){

  int k, j,m, cont, lp, i, e, f;
  double epsilon;

  cont = 0;
  epsilon = 0.00001;

  for(k = 0; k < dn; k++){
    for(j = 0; j < pn[k]-1; j++){
      if((x[k][j] < y_out[k] || x[k][j] == y_out[k]) && y_out[k] < x[k][j+1]){
	possible[k][0] = x[k][j];
	possible[k][1] = x[k][j+1];
	j = pn[k]-1;
	cont = cont + 1;
      }//printf("j: %d, k: %d, y_out[k]: %f, x[k][pn[k]-1]: %f, pn[k]: %d\n", j,k, y_out[k], x[k][pn[k]-1], pn[k]);
      if((y_out[k] > x[k][pn[k]-1] - epsilon) && (y_out[k] < x[k][pn[k]-1] + epsilon) && j == pn[k]-2){
         //   printf("in if\n");
       possible[k][0] = x[k][pn[k]-2];
	   possible[k][1] = x[k][pn[k]-1];
	   j = pn[k]-1;
      }
    }
  }
  j = 0;
  m = 0;
  lp = 0;
  for (i = 0; i < dn; i++){
    pdp[i] = 0;
  }
  create_pos(j, m, dn, pnp, pdp,possible, possible_nodes, index_nodes);/*
  printf("possible\n");
  for(e = 0; e < dn; e++){
        printf(" %f, %f \n", possible[e][0], possible[e][1]);
    //printf(" %f ", possible_nodes[i]);
  }*/
}

/*
distance a = 1: between points y_out and nodo[ind]
         a = 0: norm of nodo[ind]
*/
double distance(double *y_out, int ind, int dn, int a, double **nodo_aux){

	double sum, aux0, aux1;
	int k;
	if(a == 0){
	sum = 0;
	for(k = 0; k < dn ; k ++){
		aux0 = nodo[ind][k];
		sum = pow(aux0,2) + sum;
	}
	aux1 = sqrt(sum);
	return aux1;
	}
	if(a == 1){
	sum = 0;
	for(k = 0; k < dn ; k ++){
		aux0 = y_out[k]-nodo[ind][k];
		sum = pow(aux0,2) + sum;
	}
	aux1 = sqrt(sum);
	return aux1;
    }
    if(a == 2){//yout=noso_aux
	sum = 0;
	for(k = 0; k < dn ; k ++){
		aux0 = nodo_aux[ind][k];
		sum = pow(aux0,2) + sum;
	}
	aux1 = sqrt(sum);
	return aux1;
    }
}

/*
 function min_distance retorna los dn + 1 nodos mas cercanos a y_out
*/
/*
int *min_distance(int *neigh_nodes, int *index_nodes, double *y_out, int dn){
int  i, j, s;
double aux0, aux1;

for(i = 0; i < lp[0]; i++){

   if (i < dn + 1){
       neigh_nodes[i] = index_nodes[i];
   }
   else{    	                   //mayor a menor
       for(j = 0; j < dn +1 ; j++){
    	   if(j == 0){
		      aux0 = distance(y_out,neigh_nodes[j],dn,1);
		  }
		  else{
		  	aux1 = distance(y_out,neigh_nodes[j],dn,1);

		  	if(aux0 < aux1){
		  		aux0 = neigh_nodes[j];
		  		aux1 = neigh_nodes[0];
		  		neigh_nodes[j] = (int)(aux1);
		  		neigh_nodes[0] = (int)(aux0);
			  }
		  }
	    }
	    //comparar
       for(s = 0; s < dn+1; s++){
       	  aux0 =  distance(y_out,neigh_nodes[s], dn,1);
       	  aux1 = distance(y_out,index_nodes[i], dn,1);
	      if(aux0 > aux1){
		       neigh_nodes[s] = index_nodes[i];
		       s = dn + 1;
		  }
	   }
	}
}
}
*/
double *tras(double *y_out, int *index_nodes, double *yout_aux, int dn, double *piv){

int j, k, i;
//pivote election first iteration
for(j = 0; j < dn; j++){
    piv[j] = nodo[index_nodes[0]][j];
}/*
printf("index\n");
for(i = 0; i < 4; i++){
  printf(" %d \n",index_nodes[i]);
}printf("\n");*/

/*
printf("piv\n", piv[0], piv[1]);*/
//translation to the unit square
for(k = 0; k < dn; k++){
    yout_aux[k] = (y_out[k]- piv[k])*10;
//for(i = 0; i < pow(2,dn); i++){
//    nodo_aux[i][k] = (nodo[index_nodes[i]][k] - piv[k])*10;
//}
}
/*
printf("yout\n");
for(i = 0; i < dn ; i++){
    printf("%f,", yout_aux[i]);
}
printf("\n");
printf("nodo aux\n");
for(j = 0; j < pow(2,dn); j++){
for(i = 0; i < dn ; i++){
    printf("%f,", nodo_aux[j][i]);
}printf("\n");
}
printf("\n");
*/
}

int isnode(double *yout_aux, int dn){

int i, aux, value;

aux = 0;
for(i = 0; i < dn; i ++){
  if((0.99999 < yout_aux[i] && yout_aux[i] < 1.00001) || (yout_aux[i] == 0)) {
    aux = aux + 1;
  }
}//printf("aux is node: %d",aux);
  if(aux == dn){
    value = 1;
    return value;
  }
  if(aux < dn){
    value = 0;
    return value;
  }
}

int axis(double *yout,int dn){

int i, aux;
double epsilon;

epsilon = 0.00001;
aux = 0;
for(i = 0; i < dn; i++){
   if((yout[i] < 1 + epsilon) && (yout[i] > 1 - epsilon)){
      aux = 1;
   }
}

return aux;

}

double *is_in_axis(double *yout,int dn ,int *k_i,int *cont,int *ind,double **neigh_nodes){

int i, j;
double epsilon;

epsilon = 0.00001;

for(i = 0; i < dn; i++){
   if((yout[i] < 1 + epsilon) && (yout[i] > 1 - epsilon)){
        for(j = 0; j < dn; j++){
            neigh_nodes[cont[0]][j] = neigh_nodes[cont[0]-1][j];
        }
        neigh_nodes[cont[0]][i] = 1;
        k_i[ind[0]] = i;
        ind[0] = ind[0] + 1;
        cont[0] = cont[0] + 1;
   }
}

}

double *smaller(double **neigh_nodes,int *k_i, int *cont, int dn, int *ind, double *yout){

int i, aux;

aux = 0;
for(i = 0; i < dn; i ++){
    if(k_i[i] == -1){
        aux = aux + 1;
    }
}
//printf("aux: %d\n", aux);
if(aux == dn){
    if(cont[0] == 0){
           // printf("cont[0] in if smaller: %d\n", cont[0]);
         //is_in_axis(yout,dn,k_i,cont,ind,neigh_nodes);
        for(i = 0; i < dn; i++){
            neigh_nodes[cont[0]][i] = 0;
        }
        cont[0] = cont[0] +1;
        if(axis(yout,dn) == 1){
        is_in_axis(yout,dn,k_i,cont,ind,neigh_nodes);
        }
    }
}
else{
    for(i = 0; i < dn; i ++){
        neigh_nodes[cont[0]][i] = neigh_nodes[cont[0] -1][i];
        if(i == k_i[ind[0] - 1]){
            neigh_nodes[cont[0]][i] = 1;
        }
    }
    cont[0] = cont[0] + 1;
}
//printf("in smaller\n");
//printf("neigh_nodes[%d]: (%f,%f,%f)\n", cont[0]-1, neigh_nodes[cont[0]-1][0], neigh_nodes[cont[0]-1][1], neigh_nodes[cont[0]-1][2]);
}

double *maxim(double *yout_aux, int *k_i, int *ind,int dn){

int i, j,  aux1;
double aux0;
//max coordinate
//printf("In max\n");
for(j = 0; j < dn; j++){
   if(yout_aux[j] < 0.99999){
      aux0 = yout_aux[j];
      aux1 = j;
      j = dn;
      //printf("aux0: %f, aux1: %d\n", aux0, aux1);
   }
}

for(i = 0; i < dn; i++){
  if((aux0 < yout_aux[i]) && (yout_aux[i] < 0.99999)){
    aux0 = yout_aux[i];
    aux1 = i;
   // printf("aux0: %f, aux1: %d\n", aux0, aux1);
  }
}
k_i[ind[0]] = aux1;
//printf("k_i[%d]: %d\n", ind[0], aux1);
}

double *projection(double *yout_aux, int *k_i, int *ind, int dn){

int i;
double aux;
//printf("yout_premoved: (%f,%f,%f)\n", yout_aux[0], yout_aux[1], yout_aux[2]);
//printf("yout_aux[k_i[ind[0]]]: %f\n", yout_aux[k_i[ind[0]]]);
aux = yout_aux[k_i[ind[0]]];
for(i = 0; i < dn; i++){
    if(yout_aux[i] < 0.999999){
        yout_aux[i] = yout_aux[i]/aux;
        //printf("i: %d\n", i);
        //printf("yout_aux[%d]: %f\n",i, yout_aux[i]);
    }
}
//printf("k_i[ind[0]]: %d\n", k_i[ind[0]]);
//printf("yout_moved: (%f,%f,%f)\n", yout_aux[0], yout_aux[1], yout_aux[2]);

}

int NoOne(double *neigh_nodes, int dn){

int i, aux;

aux = 0;
for(i = 0; i < dn; i ++){
    if(neigh_nodes[i] < 1.0001 && 0.9999 < neigh_nodes[i]){
        aux = aux + 1;
    }
}

if(aux == dn){
    return 0;
}
else{
    return 1;
}
}

int *savek(int *k_i,int ind, int dn, double *neigh_nodes,int a){

int i, j, aux, cont, k, m;

if(a == 0){
for(i = 0; i < dn; i++){
        cont = 0;
        if(neigh_nodes[i] == 0){
        for(j = 0; j < dn; j++){
            if(i != k_i[j]){
                        cont = cont + 1;
                    }
        }
        //printf("cont in a== 1: %d\n", cont);
        if (cont == dn){
                    aux = i;
                    i = dn;
                    //printf("cont: %d, m: %d\n", cont,aux);
        }
           /* if(neigh_nodes[i] == 0 && k_i[j] == -1){
               aux = i;
               i = dn;
            }*/
        }
}
}
//printf("before a = 1\n");
if(a == 1){
  for(m = 0; m < dn; m++){
        cont = 0;
        for(k = 0; k < dn; k++){
                if(m != k_i[k]){
                        cont = cont + 1;
                    }
        }
        //printf("cont in a== 1: %d\n", cont);
        if (cont == dn){
                    aux = m;
                    m = dn;
                   // printf("cont: %d, m: %d\n", cont,aux);
        }

 }
}

k_i[ind] = aux;
//printf("ind: %d,k_i[%d]: %d\n", ind, ind, k_i[ind]);
}

int NoZeros(double **neigh_nodes, int *cont, int dn){
//retunr 1 if negh nodes contain Ov
int i, j, aux, aux1;

for(i = 0; i < cont[0]; i++){
    aux1 = 0;
    for(j = 0; j < dn; j++){
        if(neigh_nodes[i][j] == 0){
            aux1 = aux1 + 1;
        }
    }
    if(aux1 == dn){
        aux = 1;
        i = dn + 1;
    }
    else{
        aux = 0;
    }
}

return aux;
}

double *smaller_nodes(double **neigh_nodes,int dn,int *ind, int *cont, int *k_i){

int j, i;

for(j = ind[0]; j < dn; j++){
   if(NoOne(neigh_nodes[cont[0]-1],dn) == 1){
       //printf("k_i:(%d,%d,%d),j: %d, neigh[%d]: (%f,%f,%f)\n", k_i[0], k_i[1], k_i[2],j,cont[0]-1,neigh_nodes[cont[0]-1][0],neigh_nodes[cont[0]-1][1],neigh_nodes[cont[0]-1][2]);
       savek(k_i,j,dn,neigh_nodes[cont[0]-1],0);
    for(i = 0; i < dn ; i++){
        neigh_nodes[cont[0]][i] = neigh_nodes[cont[0]-1][i];
        if(i == k_i[j]){
            neigh_nodes[cont[0]][i] = 1;
        }
    }
    cont[0] = cont[0] + 1;
   }
   else{
       savek(k_i,j,dn,neigh_nodes[cont[0]-1],1);
       for(i = 0; i < dn; i ++){
         neigh_nodes[cont[0]][i] = 0;
      }
      if(NoZeros(neigh_nodes,cont,dn) == 1){
      for(i = 0; i < dn ; i++){
       // neigh_nodes[cont[0]][i] = neigh_nodes[cont[0]-1][i];
        if(i == k_i[j] ){
            neigh_nodes[cont[0]][i] = 1;
        }
    }
    }
    cont[0] = cont[0] + 1;
   }
}
}

double *complete_neigh(double **neigh_nodes, int *cont, int *k_i, int dn, int *ind){

int i, j;
while(cont[0] < dn + 1){
        smaller_nodes(neigh_nodes,dn,ind,cont,k_i);
}/*
for(i = 0; i < dn ; i++){
    printf("%f,", (y_out[i]-piv[i])*10);
}*/
//printf("\n");

/*
printf("neigh in comlete neigh\n");
for(j = 0; j < dn + 1; j++){
for(i = 0; i < dn ; i++){
    printf("%f,", neigh_nodes[j][i]);
}printf("\n");
}
printf("\n");
printf("k_i in comlete neigh\n");
for(j = 0; j < dn ; j++){
    printf("%d,", k_i[j]);
}printf("\n");*/

}

int *selection(int *cont, double *yout_aux, double **neigh_nodes, int *k_i, int *ind, int dn){

int i;

if(isnode(yout_aux,dn) == 1){
   //printf("yout nodo\n");
    for(i = 0; i < dn ; i++){
    //printf("%f,", yout_aux[i]);
}//printf("\n");
        for(i = 0; i < dn; i++){
                    neigh_nodes[cont[0]][i] = yout_aux[i];

        }
       /*  printf("neigh_nodes[%d][]\n", cont[0]);
    for(i = 0; i < dn ; i++){
    printf("%f,", neigh_nodes[cont[0]][i]);
}printf("\n");*/

        cont[0] = cont[0] + 1;
        //printf("cont actualizado: %d\n", cont[0]);

        complete_neigh(neigh_nodes,cont,k_i,dn,ind);
}
else{
    //printf("yout (%f,%f,%f),cont: %d, ind: %d, k_i[%d]: %d \n",yout_aux[0], yout_aux[1], yout_aux[2], cont[0], ind[0], ind[0], k_i[ind[0]]);
    smaller(neigh_nodes,k_i,cont,dn,ind,yout_aux); //put the smaller node in neigh node
    maxim(yout_aux,k_i,ind,dn); //k_i[ind] = max coordinate
    projection(yout_aux,k_i,ind,dn);
    ind[0] = ind[0] + 1;
    //printf("cont pres selection: %d\n ", cont[0]);
    selection(cont,yout_aux,neigh_nodes,k_i,ind,dn);
}
}

double *soround(double *y_out, int *index_nodes, int dn, double *piv, double *yout_aux, double **neigh_nodes, int *k_i, int *ind, int *cont){

int i, j, k, k_ind;
double aux, aux1;

for (i = 0; i < dn; i++){
    k_i[i] = -1;
}
ind[0] = 0;
cont[0] = 0;
tras(y_out,index_nodes,yout_aux,dn, piv);
//printf("pre select, yout: (%f,%f,%f), yout_tras: (%f,%f,%f)\n", y_out[0], y_out[1], y_out[2], yout_aux[0], yout_aux[1], yout_aux[2]);
selection(cont,yout_aux,neigh_nodes,k_i,ind,dn);
//printf("fin selection\n");

}

double *original(double **neigh_nodes, double *piv, double **nodo_aux,int dn){

int i, j, cont, aux, k;
double aux1, aux2;

for(i = 0; i < dn + 1; i++){
        cont = 0;
    for(j = 0; j < dn; j++){
        if(neigh_nodes[i][j] == 0){
            cont = cont + 1;
        }
    }
    if((cont == dn) && (i > 0)){
        aux = i;
        for(k = 0; k < dn; k ++){
          aux1 = neigh_nodes[0][k];
          aux2 = neigh_nodes[aux][k];
          neigh_nodes[0][k] = aux2;
          neigh_nodes[aux][k]= aux1;
          //printf("aux1: %f, aux2: %f, k: %d, neig0: %f, neigha: %f, aux: %d \n", aux1, aux2, k, neigh_nodes[0][k], neigh_nodes[aux][k], aux);
        }
    }
}

for(i = 0; i < dn + 1; i++){
    for(j = 0; j < dn; j ++){
        nodo_aux[i][j] = (neigh_nodes[i][j]/10) + piv[j];
    }
}
/*
printf("yout\n");
for(i = 0; i < dn ; i++){
    printf("%f,", y_out[i]);
}
printf("\n");
printf("nodo aux\n");
for(j = 0; j < dn+1; j++){
for(i = 0; i < dn ; i++){
    printf("%f,", nodo_aux[j][i]);
}printf("\n");
}
*/
/*
printf("nodo neigh\n");
for(i = 0; i < dn ; i++){
for(j = 0; j < dn+1; j++){
    printf("%f,", neigh_nodes[j][i]);
}printf("\n");
}
printf("\n");
*/
}

double *a_and_b(double *new_a, double *new_b, double *y_out, double **nodo_aux, int dn){

int i, j, aux;

for(i = 0; i < dn; i++){
    new_b[i] = y_out[i] - nodo_aux[0][i];
}

aux = 0;
for(i = 0; i < dn; i++){
    for(j = 1; j < dn + 1; j++){
        new_a[aux] = nodo_aux[j][i]-nodo_aux[0][i];
        aux = aux + 1;
    }
}

/*
printf("nodo aux\n");
for(i = 0; i < dn ; i++){
for(j = 0; j < dn+1; j++){
    printf("%f,", nodo_aux[j][i]);
}printf("\n");
}
printf("nodo aux-nodo piv\n");
for(i = 0; i < dn ; i++){
for(j = 1; j < dn+1; j++){
    printf("%f,", nodo_aux[j][i]-nodo_aux[0][i]);
}printf("\n");
}
printf("new_a\n");
for(j = 0; j < dn*dn; j++){
    printf("%f,", new_a[j]);
}
printf("\n");*/
}

double *convex(double *a_data, double *b_data, double *lambdas, int dn){

  int i;
  double sum;

  gsl_matrix_view m;
   m    = gsl_matrix_view_array (a_data, dn, dn);

  gsl_vector_view b    = gsl_vector_view_array (b_data, dn);

  gsl_vector *ex = gsl_vector_alloc (dn);

  int s;

  gsl_permutation * p = gsl_permutation_alloc (dn);

  gsl_linalg_LU_decomp (&m.matrix, p, &s);

  gsl_linalg_LU_solve (&m.matrix, p, &b.vector, ex);

  //printf ("x = \n");
  sum = 0;
  for(i = 0; i < dn ; i++){
    lambdas[i] = ex ->data[i];
    sum = sum + lambdas[i];
  }
  lambdas[dn] = 1 - sum;
/*
  for(i = 0; i < dn + 1; i ++){
      printf("lamb[%d]: %f\n",i,lambdas[i]);
  }*/

  //gsl_vector_fprintf (stdout, x, "%g");

  gsl_permutation_free (p);
  gsl_vector_free (ex);
  //while(1);
  return 0;
}
/*
**nodo, **control, *u_sol

*/

int *searching_nodes(double **nodo_aux, int *index_nodes, int dn, int *index){

int i, j, k, cont;
double epsilon;

epsilon= 0.0001;

for(i = 0; i < dn + 1; i ++){
    for(k = 0; k < pow(2,dn); k++){
            cont = 0;
            for(j = 0; j < dn; j++){
                if(nodo_aux[i][j] < (nodo[index_nodes[k]][j] + epsilon) && nodo_aux[i][j] > (nodo[index_nodes[k]][j] - epsilon)){
                    cont = cont + 1;
                }
            }
            if(cont == dn){
                index[i] = index_nodes[k];
               // k = pow(2,dn);
            }
    }
}
}

double *cost_function(int pos_n, int pos_c, int dn, double *efe){

int i;
double aux0, aux1, sum;

sum = 0;
for(i = 0; i < dn ; i ++){
		aux0 = nodo[pos_n][i];
		aux1 = pow(aux0,2);
		sum = aux1 + sum;
	}
	efe[0] = sqrt(sum);

}

void push_T(T_cal_t * lis, double aux){

T_cal_t * current =lis;

 while (current->nex != NULL) {
        current = current->nex;
    }
    current->nex = calloc(1,sizeof(T_cal_t));
    //current->next_inf = calloc(1,sizeof(adm_control_t));
    current->nex->value = aux;
    //current->next_inf = NULL;
    current->nex->nex = NULL;
   // printf("number node: %d\n", current->next_n->n);
}

double *T(T_cal_t *lis, double *efe, double h, int c, int k, double rate, double *lambdas, int *ind, int dn, int cont, double *TU, double *u_sol, int option){
//$(T(u))_{i} = min_{a \in A:xi}{(1-lamb*h) \lambda(a) u + hF(a))_{i}}$

int i;
double sum, aux0, aux1;

sum = 0;


if(option == 1){
for(i = 0; i < dn + 1; i++){
           sum = lambdas[i]*u_sol[ind[i]] + sum;
        }
  aux0 = (1- rate*h)*sum + h*efe[0];
  if(cont == 0){
   lis->value =aux0;
   lis->nex = NULL;
}
else{
  push_T(lis,aux0);
}
}
if(option == 0){

if(cont == 0){
        for(i = 0; i < dn + 1; i++){
           sum = lambdas[i]*u_sol[ind[i]] + sum;
        }
  aux0 = (1- rate*h)*sum + h*efe[0];
}
else{
        for(i = 0; i < dn + 1; i++){
           sum = lambdas[i]*u_sol[ind[i]] + sum;
        }
  aux1 = (1- rate*h)*sum + h*efe[0];
  if(aux1 < aux0){
    aux0 = aux1;
  }
}
TU[k] = aux0;
}
}

double norm(double *TU, double *u_sol){

int i;
double sum, aux;

sum = 0;
for (i = 0; i < l[0]; i ++){
    sum = pow((TU[i]-u_sol[i]),2)+sum;
}

aux = sqrt(sum);
return aux;
}

double *minimum(T_cal_t *lis,int nody, int dn, double *TU){

    T_cal_t * current = lis;
    double aux, aux1;
    int cont;

    cont = 0;
    while(current != NULL){
            if(cont == 0){
        aux = current->value;
            }
            else{
        aux1 = current->value;
        if(aux1 < aux){
            aux = aux1;
        }
            }
            current = current->nex;
        cont = cont + 1;
    }
    TU[nody] = aux;

}

/*
 head is a structure containing val (control), lambdas[1..dn+1], ind[1...dn+1](associated nodes to each lambda)
 k = node indicator
 efe = pointer which save the value of l(x_i,a);
*/
double *fix_point(nodes_inf_t * body, int k, int dn, double h, double rate, double *efe, double *TU, double *u_sol){
   // $(T(u))_{i} = min_{a \in A:xi}{(1-lamb*h) \lambda(a) u + hF(a))_{i}}$
   //F(a) = l(xi,a);
   int c, cont, nody, aux, aux1,i, option;
   double epsilon;
   T_cal_t *lis;

   aux = 0;
   aux1 = 0;
   epsilon = 0.0001;
   option = 0;
   do{
   nodes_inf_t * current = body;
   cont = 0;
   while (current != NULL) {
       //printf("lis\n");
         lis = NULL;
         lis = calloc(1,sizeof(T_cal_t));
         //lis = calloc(1,sizeof(T_cal_t));
        nody = current->n; //select a node
       // printf("nody: %d\n", nody);
        //if(current->next_inf == NULL){
        //    printf("fail\n");
        //}
        cont = 0;
        if(current->next_inf->next != NULL){//ensure A_h(x_i) is not empty
                adm_control_t * current1 = current->next_inf;
             while(current1 != NULL){//calculetes T(U)=min{...} for every control c
                 c = current1->val;
                 cost_function(nody,c,dn, efe); //efe[0] the value
                 //printf("nody: %f, %f, efe: %f\n", nodo[nody][0], nodo[nody][1], efe[0]);
                 T(lis,efe,h,c,nody,rate,current1->lambda,current1->ind,dn,cont,TU,u_sol,option);
                 current1 = current1->next;
                 cont = cont + 1;
            }
        }
        else{//when A_h(x_i) is empty
                if(option == 1){
                if(cont == 0){
                    lis->value = 100;
                    lis->nex = NULL;
                }
                else{
                push_T(lis,100);
                }
                }
                if(option == 0){
                    TU[nody] = 10;

                }

        } //end while control
        if(option == 1){
        minimum(lis,nody,dn,TU);
     free(lis);
        }
   current = current->next_n;
   }//end while nodes
   aux = aux + 1;
   if(norm(TU,u_sol) < epsilon){
    aux1 = 1;
    printf("Norm %f\n", norm(TU,u_sol));
    printf("aux: %d", aux);
   }
   else{
    for(i = 0; i < l[0]; i++){
        u_sol[i] = TU[i];
    }
   }
  }while( (aux1 < 1) && (aux < 100000));  //comparison T_U, u_sol
}

void test(nodes_inf_t * body, adm_control_t * head, int dn){
    int cont;
    cont= 0;
while(cont < 3){
   //print_list_controls(head,dn);
    print_list_test(body,dn);
    printf("cont: %d", cont);
    cont = cont + 1;
}
   printf("cont: %d", cont);

}

double *optimal_solution(void *o, double *y_out, int *pn, int *pnp, int *pd, int *pdp, double **possible,  double *possible_nodes, double **neigh_nodes, int *index_nodes, double *new_a,double *new_b, double *lambdas, double *piv, double **nodo_aux, double *yout_aux, int *k_i, int *ind, int *counter, double *efe, int *index, double *TU, double *u_sol){

  int dc, dn, cont,g;
  double h;
  double sum;
  int j, k, s, m, n, i, aux, jj, e, auxi;
  struct posiciones *t;

  t = (struct posiciones *)o;
  s = t->fi;
  m = t->fs;
  dn = t->dimn;
  dc = t->dimc;
  h = t->hh;
  adm_control_t * head;
  nodes_inf_t * body;

  body = NULL;
  body = calloc(1,sizeof(nodes_inf_t));
  auxi = 0;

  for(k = s ; k < m  ; k++){
/*
    printf("Node\n");
    for(e = 0; e < dn; e++){
        printf(" %f, ", nodo[k][e]);
    }printf("\n");*/
    if(auxi == 0){
        body->n = k;
        body->next_n = NULL;
        body->next_inf = NULL;//malloc(sizeof(adm_control_t));
    }
    else{
        push_node(body,k);
    }
    cont = 0;
    head = NULL;
    head = calloc(1,sizeof(adm_control_t));
    //head->next = NULL;
    for(j = 0; j < lc[0]; j++){
	aux = 0;
	f(y_out, k, j, h, dc, dn);//returns y= x+ h*f, for each particular case, f must be changed in this function
            //printf("yout: %f, %f, node: %f, %f, control: %f, %f \n", y_out[0], y_out[1], nodo[k][0], nodo[k][1], control[j][0]);
            //while(1);
	aux = in_region(y_out,dn,pn);
	if(aux == 1){
	  lp[0] = 0;/*
	   printf(" yout\n ");
      for(i = 0; i < dn ; i++){
          printf(" %f,", y_out[i]);
      }printf("\n");
      printf("\n");*/
	  neighbourhood(possible, possible_nodes ,y_out, dn, pn, pnp, pd, pdp, index_nodes);
	  soround(y_out,index_nodes, dn, piv, yout_aux, neigh_nodes,k_i,ind,counter);
	   /* printf("piv\n");
      for(i = 0; i < dn ; i++){
          printf("%f,", piv[i]);
      }
      printf("\n");*/
	  original(neigh_nodes,piv,nodo_aux,dn);//return dn+1 nodes for linear covnex combination_ nodo_aux
	  searching_nodes(nodo_aux,index_nodes,dn,index);
	  a_and_b(new_a,new_b,y_out,nodo_aux,dn);
	  convex(new_a,new_b,lambdas,dn);
      if(cont == 0){
        head->val = j;
        head->lambda = calloc(dn + 1,sizeof(double));
        head->ind = calloc(dn + 1,sizeof(int));
        for(e = 0; e < dn + 1; e++){
            head->lambda[e] = lambdas[e];
        }
        for(e = 1; e < dn + 1; e++){
            head->ind[e-1] = index[e];
        }
        head->ind[dn] = index[0];
        head->next = NULL;
	  }
	  else{/*
            for(e = 0; e < dn+1; e++){
               printf("lamb[%d]: %f", e, lambdas[e]);
            }printf("\n");*/
	  push(head, j, lambdas, dn, index);//add node indices
	  }

/*
printf("nodes neigh\n");
for(jj = 0; jj < dn + 1; jj++){
for(i = 0; i < dn ; i++){
    printf("%f,", neigh_nodes[jj][i]);
}printf("\n");
}
printf("\n");
printf("k_i\n");
for(i = 0; i < dn ; i++){
    printf("%d,", k_i[i]);
}
printf("\n");
*/
     cont = cont + 1;
    }//end if in region
    }//end for controls
    auxi = auxi + 1;
   //print_list_controls(head,dn);
     push_nodes_controls(head,body,dn);
    }//end for nodes

   //print_list_nodes_and_controls(body,dn);

   //test(body,head,dn);
       //rate = 1;
              /*
             int auxiliar;
            auxiliar = 0;
            adm_control_t * aux_list;
            while(auxiliar < 3){
                    aux_list = NULL;
                aux_list = head;
                test(aux_list,dn);
                printf("%d", auxiliar);
                auxiliar = auxiliar + 1;
            }
            while(1);*/
          fix_point(body,k,dn,h,1,efe,TU,u_sol);

  //*pthread_exit(0);
}

/* Entrada: u  -> cantidad de sub problemas (para implementar con thread)
             daton
             datoc
             dn     -> dimension del espacio de estados
             dc     -> dimension del espacio de control

   s      -> min column para thread
   p      -> max column para thread
   y_sol  -> guarda el valor  xi+hf();
   */

int main(int argc, char *argv[]){

   double h, kn, kc, ini;
   int i, j, n, w, k, t, u;
   int s, p;
   int dn, dc, m, aux;
   int *pd, *pn, *pc,*pdc, *pnp, *pdp, **k_i, **ind, **counter;
   int **index_nodes, **index;
   double **y_out, ***possible, **possible_nodes;
   double **piv, ***nodo_aux, **yout_aux, ***neigh_nodes;
   pthread_t *thread;
   pthread_attr_t attribute;
   void *exit_status;
   struct posiciones **pos;
   double **new_a,**new_b, **efe;
   double **lambdas, **TU, **u_sol;
   h = 0.1;
   kn = 0.1;
   kc = 0.1;
   dn = 2;
   dc = 2;
   u = 1;

	//*scanf("%f",&h); //* time discretization
	//*scanf("%f", &k1);//* space discretization
	//*scanf("%f",&k2);
	//*printf("k1: %f", k1);
	//*scanf("%d",&d1); //*dimension del espacio de estados

	pd = calloc(dn,sizeof(int ));
	pdc = calloc(dc,sizeof(int ));
	pdp = calloc(dn,sizeof(int ));

	daton = calloc(dn,sizeof(double *));
	for(i = 0; i < dn ;i++){
	  daton[i] = calloc(2,sizeof(double));
	}

	/*
	i = 0;
	while (i < d1 ) {
      j = 0;
      while (j < 2 ) {
         scanf("%f",&dato1[i][j]); //* por cada i guarda el valor inferior y superior
         j = j + 1;
      }
      i = i + 1;
    }
	*/

//2d matlab
    if(dn == 2){
	daton[0][0] = -2;
	daton[0][1] = 2;
	daton[1][0] = -2;
	daton[1][1] = 2;
    }
	//3d
	if(dn == 3){
	daton[0][0] = 0;
	daton[0][1] = 0.5;
	daton[1][0] = 0;
	daton[1][1] = 1;
	daton[2][0] = 0;
	daton[2][1] = 0.5;
	}
//*	scanf("%d",&d2); //*dimension del espacio de controles
	datoc = calloc(dc,sizeof(double *));
	for(i = 0; i < dc ;i++){
      datoc[i] = calloc(2,sizeof(double));
    }

	/*
	i = 0;
	while (i < d2 ) {
      j = 0;
      while (j < 2 ){
         scanf("%f",&dato2[i][j]); //* por cada i guarda el valor inferior y superior
         j = j + 1;
       }
       i = i + 1;
   }
   */
 //2d matlab
    if(dn == 2){
            dc = 1;
    datoc[0][0] = -1;
	datoc[0][1] = 0;
	//datoc[1][0] = -1;
	//datoc[1][1] = 0;
    }

	//3d
	if(dn == 3){
	datoc[0][0] = 0.1;
	datoc[0][1] = 0.5;
	datoc[1][0] = 0;
	datoc[1][1] = 0.5;
	datoc[2][0] = 0;
	datoc[2][1] = 0.5;
	}

	pos = calloc(u,sizeof(struct posiciones *));
	thread = calloc(u,sizeof(pthread_t));
    pthread_attr_init(&attribute);
    pthread_attr_setdetachstate(&attribute,PTHREAD_CREATE_JOINABLE);
    for(i = 0; i < u; i++){
      pos[i] = calloc(1, sizeof(struct posiciones));
    }

	pn = calloc(dn,sizeof(int )); //guarda el largo de cada dimension
	pnp = calloc(dn,sizeof(int )); //guarda el largo de los putos vecinos en cada dimeension
	x = calloc(dn,sizeof(double *));
	for(i = 0; i < dn ; i++){
		pn[i] = (int)((daton[i][1]-daton[i][0])/kn) + 1;
		x[i]= calloc(pn[i],sizeof(double));
		pnp[i] = 2;
	}

	pc = calloc(dc,sizeof(int ));
	a = calloc(dc, sizeof(double *));
	for(i = 0; i < dc; i++){
		pc[i] = (int)((datoc[i][1]-datoc[i][0])/kc) + 1;
		a[i]= calloc(pc[i],sizeof(double));
	}
	//printf("pc: %d", pc[1]);
	l = calloc(1,sizeof(int ));
	l[0] = 1;
	for(i = 0; i < dn ; i++){
		aux = l[0];
		l[0] = (pn[i])*aux;
	}
	nodo = calloc(l[0],sizeof(double *));
	for(i = 0; i < l[0] ; i++){
		nodo[i] = calloc(dn,sizeof(double ));
	}
	lc = calloc(1,sizeof(int ));
	lc[0] = 1;
	for(i = 0; i < dc ; i++){
		aux = lc[0];
		lc[0] = (pc[i])*aux;
	}
	control = calloc(lc[0],sizeof(double *));
	for(i = 0; i < lc[0] ; i++){
		control[i] = calloc(dc,sizeof(double ));
	}

	for(i = 0; i < dn; i++){
		ini= daton[i][0];
		for(j = 0; j < pn[i] ; j++){
			x[i][j]= ini+(kn*j);
	    }
	}/*
	for(i = 0; i < dn; i++){
		for(j = 0; j < pn[i] ; j++){
                printf(" %f ",x[i][j]);
	    }printf("\n");
	}printf("\n");*/


	for(i = 0; i < dc; i++){
	    ini= datoc[i][0];
	    for(j = 0; j < pc[i] ; j++){
			a[i][j]= ini+(kc*j);
	    }
	}

	l[0] = 0;
	j = 0;
	m = 0;

	for (i = 0; i < dn; i++){
		pd[i] = 0;
	}

	for (i = 0; i < dc; i++){
	  pdc[i] = 0;
	}

	for (i = 0; i < dn; i++){
	  pdp[i] = 0;
	}

    create(j,m,dn,pn,pd);

    lc[0]=0;
    j = 0;
	m = 0;

    create_c(j,m,dc,pc,pdc);

    //printf("ele: %d",l[0]);

	k = 0;

    y_out =  calloc(u,sizeof(double *));
    //valid_controls = malloc(sizeof(adm_control_t));
    //lamdba_valid_controls = malloc(sizeof(lambda_t));
    possible = calloc(u,sizeof(double **));
    possible_nodes = calloc(u,sizeof(double *));
    index_nodes = calloc(u,sizeof(int *));
    neigh_nodes = calloc(u,sizeof(double **));
    piv = calloc(u,sizeof(double *));
    nodo_aux = calloc(u,sizeof(double *));
    yout_aux = calloc(u,sizeof(double *));
    k_i = calloc(u,sizeof(int *));
    ind = calloc(u,sizeof(int *));
    counter = calloc(u,sizeof(int *));

    lp = calloc(1,sizeof(int ));
//    x_piv=calloc(dn,sizeof(double ));
    lambdas = calloc(u,sizeof(double *));
    new_a= calloc(u,sizeof(double *));
    new_b= calloc(u,sizeof(double *));
    efe = calloc(u,sizeof(double *));
    index = calloc(u,sizeof(int *));
    TU = calloc(u,sizeof(double *));
    u_sol = calloc(u,sizeof(double *));

    //lp[0] = 0;
    for(i = 0; i < u ; i++){
        TU[i] = calloc(l[0],sizeof(double ));
        u_sol[i] = calloc(l[0],sizeof(double ));
        k_i[i] = calloc(dn, sizeof(int ));
        ind[i]  = calloc(1, sizeof(int ));
        counter[i]  = calloc(1, sizeof(int ));
    	y_out[i] = calloc(dn,sizeof(double ));
    	possible[i] = calloc(dn,sizeof(double *));
    	neigh_nodes[i] = calloc(dn + 1,sizeof(double *));
    	possible_nodes[i] = calloc(pow(2,dn),sizeof(double ));
    	index_nodes[i] = calloc(pow(2,dn),sizeof(int ));
    	index[i] = calloc(dn + 1,sizeof(int ));
        piv[i] = calloc(dn,sizeof(double ));
        nodo_aux[i] = calloc(dn+1,sizeof(double *));
        yout_aux[i] = calloc(dn,sizeof(double ));
        lambdas[i] = calloc(dn+1,sizeof(double ));
        new_a[i] = calloc(dn*dn,sizeof(double ));
        new_b[i] = calloc(dn,sizeof(double ));
        efe[i] = calloc(1, sizeof(double ));
    	for(j = 0; j < dn; j++){
    		possible[i][j] = calloc(2,sizeof(double ));
		}
		for(j = 0; j < dn + 1; j++){
    		neigh_nodes[i][j] = calloc(dn,sizeof(double ));
    		nodo_aux[i][j] = calloc(dn,sizeof(double ));
		}
         s = i*k;
         p = (int)(l[0]/u)*(i + 1);
         pos[i]->fi = s;
         pos[i]->fs = p;
         pos[i]->dimn = dn;
         pos[i]->dimc = dc;
         pos[i]->hh = h;
	     optimal_solution(pos[i],y_out[i], pn, pnp, pd, pdp, possible[i], possible_nodes[i], neigh_nodes[i],index_nodes[i], new_a[i], new_b[i], lambdas[i], piv[i], nodo_aux[i], yout_aux[i],k_i[i],ind[i],counter[i], efe[i], index[i],TU[i],u_sol[i]);
		 /*
         pthread_create(&thread[i],&attribute,optimal_solution,(void *) pos[i], (int *) y_out);
         */
         k = (int)(l[0]/u);
      }
      /*
    pthread_attr_destroy(&attribute);
    for (i = 0; i < u; i = i + 1){
      pthread_join(thread[i],&exit_status);
    }
    */


    printf("U\n");
    for(j = 0; j < u; j++){
    for(i = 0; i < l[0]; i++){
        printf("%f\n", u_sol[j][i]);
    }
    printf("\n");
    }

printf("Ireland is Green");

}

//hola gopi

