#include "mcqd.h"

Maxclique::Maxclique (const bool* const* conn, const int sz, const float tt) : pk(0), level(1), Tlimit(tt), V(sz), Q(sz), QMAX(sz) {
  assert(conn!=0 && sz>0);
  for (int i=0; i < sz; i++) V.push(i);
  e = conn;
  C = new ColorClass[sz + 1];
  for (int i=0; i < sz + 1; i++) C[i].init(sz + 1);
  S = new StepCount[sz + 1];
}

void Maxclique::_mcq(int* &maxclique, int &sz, bool dyn) { 
  V.set_degrees(*this);
  V.sort();
  V.init_colors();
  if (dyn) {
    for (int i=0; i < V.size() + 1; i++) {
      S[i].set_i1(0);
      S[i].set_i2(0);
    }
    expand_dyn(V);
  }
  else
    expand(V);
  maxclique = new int[QMAX.size()]; 
  for (int i=0; i<QMAX.size(); i++) { 
    maxclique[i] = QMAX.at(i);
  }
  sz = QMAX.size();
}

void Maxclique::Vertices::init_colors() { 
  const int max_degree = v[0].get_degree();
  for (int i = 0; i < max_degree; i++)
    v[i].set_degree(i + 1);
  for (int i = max_degree; i < sz; i++)
    v[i].set_degree(max_degree + 1);
}

void Maxclique::Vertices::set_degrees(Maxclique &m) { 
  for (int i=0; i < sz; i++) {
    int d = 0;
    for (int j=0; j < sz; j++)
      if (m.connection(v[i].get_i(), v[j].get_i())) d++;
    v[i].set_degree(d);
  }
}

bool Maxclique::cut1(const int pi, const ColorClass &A) {
  for (int i = 0; i < A.size(); i++)
    if (connection(pi, A.at(i)))
      return true;
  return false;
}

void Maxclique::cut2(const Vertices &A, Vertices &B) {
  for (int i = 0; i < A.size() - 1; i++) {
    if (connection(A.end().get_i(), A.at(i).get_i()))
      B.push(A.at(i).get_i());
  }
}

void Maxclique::color_sort(Vertices &R) {
  int j = 0;
  int maxno = 1;
  int min_k = QMAX.size() - Q.size() + 1;
  C[1].rewind();
  C[2].rewind();
  int k = 1;
  for (int i=0; i < R.size(); i++) {
    int pi = R.at(i).get_i();
    k = 1;
    while (cut1(pi, C[k]))
      k++;
    if (k > maxno) {
      maxno = k;
      C[maxno + 1].rewind();
    }
    C[k].push(pi);
    if (k < min_k) {
      R.at(j++).set_i(pi);
    }
  }
  if (j > 0) R.at(j-1).set_degree(0);
  if (min_k <= 0) min_k = 1;
  for (k = min_k; k <= maxno; k++)
    for (int i = 0; i < C[k].size(); i++) {
      R.at(j).set_i(C[k].at(i));
      R.at(j++).set_degree(k);
    }
}

void Maxclique::expand(Vertices R) {
  while (R.size()) {
    if (Q.size() + R.end().get_degree() > QMAX.size()) {
      Q.push(R.end().get_i());
      Vertices Rp(R.size());
      cut2(R, Rp);
      if (Rp.size()) {
        color_sort(Rp);
	pk++;
        expand(Rp);
      }
      else if (Q.size() > QMAX.size()) { 
        //std::cout << "step = " << pk << " current max. clique size = " << Q.size() << std::endl; 
	QMAX = Q;
      }    
      Rp.dispose();
      Q.pop();
    }
    else {
      return;
    }
    R.pop();
  }
}

void Maxclique::expand_dyn(Vertices R) {
  S[level].set_i1(S[level].get_i1() + S[level - 1].get_i1() - S[level].get_i2());
  S[level].set_i2(S[level - 1].get_i1());
  while (R.size()) {
    if (Q.size() + R.end().get_degree() > QMAX.size()) {
      Q.push(R.end().get_i());
      Vertices Rp(R.size());
      cut2(R, Rp);
      if (Rp.size()) {
        if ((float)S[level].get_i1()/++pk < Tlimit) {
          degree_sort(Rp);
        }
        color_sort(Rp);
	S[level].inc_i1();
	level++;
	expand_dyn(Rp);
	level--;
      }
      else if (Q.size() > QMAX.size()) { 
        //std::cout << "step = " << pk << " current max. clique size = " << Q.size() << std::endl; 
	QMAX = Q;
      }    
      Rp.dispose();
      Q.pop();
    }
    else {
      return;
    }
    R.pop();
  }
}
