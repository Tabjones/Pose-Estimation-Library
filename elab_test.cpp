#include <iostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <string>

using namespace std;
using namespace boost;

int k(40); //set to the k used during the tests

void
getResults (string file, vector<int>& vec)
{
  ifstream f (file.c_str());
  if (f.is_open())
  {
    string num;
    while (getline (f, num, '_'))
      vec.push_back( stoi(num) );
    f.close();
  }
  else
  {
    cout<<"Cant open test file"<<endl;
    exit (0);
  }
}

void
calcSuccess (vector<int>& vec, vector<float>& prob, int tot, int mr)
{
  prob.resize(mr);
  for (size_t i=0; i<prob.size(); ++i)
  {
    prob[i] = 0;
    for (size_t j=0; j<vec.size(); ++j)
      if ((vec[j] <= (i+1)) && (vec[j] != 0))
        prob[i]++;
    prob[i] /= tot; //get percentage
    prob[i] *= 100;
  }
}

/* MAIN*/  
int
main (int argc, char** argv)
{
  bool test_l(false);
  bool test_u(false);
  bool no_ESF(false);
  if (argc > 1)
  {
    string test = argv[1];
    if (test.compare("-tl") == 0)
    {
      cout<<"Elaboring test final candidate ...\n";
      test_l = true;
    }
    else if (test.compare("-tu") == 0)
    {
      cout<<"Elaboring unknown object test ...\n";
      test_u = true;
    }

    else if (test.compare("-no_ESF") == 0)
    {
      cout<<"excluding ESF \n";
      no_ESF=true;
    }
    else
    { 
      cout<<"Unrecognized options, only option is -tl, -tu or -no_ESF"<<endl;
      exit(0);
    }
    if (argc>2)
    {
      string noesf = argv[2];
      if (noesf.compare("-no_ESF") == 0)
        no_ESF = true;
    }
  }
  else
  {
    cout<<"Elaboring test features ...\n";
    test_l = false;
    test_u = false;
  }

  if (!test_l && !test_u)
  {
  //VFH
    vector<int> vfh_result;
    getResults ("vfh.test", vfh_result);
  //ESF
    vector<int> esf_result;
    if (!no_ESF)
      getResults ("esf.test", esf_result);
  //CVFH
    vector<int> cvfh_result;
    getResults ("cvfh.test", cvfh_result);
  //VFH
    vector<int> ourcvfh_result;
    getResults ("ourcvfh.test", ourcvfh_result);
  //Composite  
    vector<int> composite_result;
    getResults ("composite.test", composite_result);
  
  // calculate accumulated success per rank, assume max rank is 15
    int num_test = vfh_result.size();
    vector<float> vfh, esf, cvfh, ourcvfh, composite;
    calcSuccess (vfh_result, vfh, num_test, k);
    if(!no_ESF)
      calcSuccess (esf_result, esf, num_test, k);
    else
      esf.resize(k,0);
    calcSuccess (cvfh_result, cvfh, num_test, k);
    calcSuccess (ourcvfh_result, ourcvfh, num_test, k);
    calcSuccess (composite_result, composite, num_test, k);
    ofstream result ("features.txt");
    cout<<"\tVFH\tESF\tCVFH\tOURCVFH\tCOMPOSITE\n";
    result<<"\tVFH\tESF\tCVFH\tOURCVFH\tCOMPOSITE\n";
    for (int i=0; i<k; ++i)
    {
      cout<<"R"<<i+1<<"\t"<<vfh[i]<<"\t"<<esf[i]<<"\t"<<cvfh[i]<<"\t"<<ourcvfh[i]<<"\t"<<composite[i]<<"\n";
      result<<"R"<<i+1<<"\t"<<vfh[i]<<"\t"<<esf[i]<<"\t"<<cvfh[i]<<"\t"<<ourcvfh[i]<<"\t"<<composite[i]<<"\n";
    }
    result.close();
  }
  else if (test_l) //test candidates
  {
    vector<vector<int> > result; //colums num of success per position (at pos 0 failures, 1 success, 2 neigh success, 3 same obj, 4 wrong obj, at 5 num of tests, 6 underrmse, 7 overrmse)
                                 //rows objects, last row totals
    vector<string> names; //objects name
    vector<int> times; //average times per object
    ifstream f ("candidate.test");
    if (f.is_open())
    {
      string line;
      while (getline (f, line))
      {
        vector<string> vst;
        split(vst, line, boost::is_any_of("_"), boost::token_compress_on);
        int res = stoi(vst.at(2));
        int time = stoi(vst.at(3));
        int typ = stoi(vst.at(4));
        if (names.empty())
        {
          names.push_back(vst.at(0));
          vector<int> tmp (8, 0);
          tmp[res]++;
          tmp[5]++;
          if (typ!=0)
            tmp[typ]++;
          result.push_back(tmp);
          times.push_back(time);
        }
        else
        {
          bool found = false;
          for(size_t i=0; i<names.size(); ++i)
            if (names[i].compare(vst.at(0)) == 0)
            {
              found = true;
              result[i][res]++;
              if (typ!=0)
                result[i][typ]++;
              result[i][5]++;
              times[i] += time;
            }
          if (!found)
          {
            names.push_back(vst.at(0));
            vector<int> tmp (8, 0);
            tmp[res]++;
            tmp[5]++;
            if (typ!=0)
              tmp[typ]++;
            result.push_back(tmp);
            times.push_back(time);
          }
        }
      }
      f.close();
      for (size_t i=0; i<times.size(); ++i)
        times[i] /= result[i][5];
    }
    else
    {
      cout<<"Cant open test file"<<endl;
      exit (0);
    }
    ifstream ft ("GlobalTime.test");
    float aver_time = 0.0f;
    if (ft.is_open())
    {
      string line;
      int num = 0;
      while (getline (ft, line))
      {
        int time = stoi(line);
        aver_time += time;
        ++num;
      }
      aver_time /= num;
    }
    else
    {
      cout<<"Cant open test file"<<endl;
      exit (0);
    }
    ofstream candf ("candidate.txt");
    candf<<"Name\tFail\tSucc\tDirN\tSameObj\tFalse\tNumTest\tUrmse\tOrmse\tAvrgTime\n";
    cout<<"Name\tFail\tSucc\tDirN\tSameObj\tFalse\tNumTest\tUrmse\tOrmse\tAvrgTime\n";
    for(size_t i=0; i< names.size(); ++i)
    {
      candf<<names[i].c_str()<<"\t";
      cout<<names[i].c_str()<<"\t";
      for (size_t j=0; j<8; ++j)
      {
        candf<<result[i][j]<<"\t";
        cout<<result[i][j]<<"\t";
      }
      candf<<times[i]<<endl;
      cout<<times[i]<<endl;
    }
    cout<<endl;
    candf<<endl;
    cout<<"Total Average Time\t"<<aver_time<<endl;
    candf<<"Total Average Time\t"<<aver_time<<endl;
    candf.close();
  }
  else if (test_u)
  {
    string name("nothing"); //object name
    vector<string> results; //candidates name
    vector<int> howmany;
    int total (0);
    ifstream f ("unknown.test");
    if (f.is_open())
    {
      string line;
      while (getline (f, line))
      {
        vector<string> vst;
        split(vst, line, boost::is_any_of("_"), boost::token_compress_on);
        if (vst[2].compare("NoCandidate") == 0)
          continue;
        int a = stoi(vst[1]);
        int b = stoi(vst[3]);
        string cand = vst[2];
        if (total==0)
        {
          name =vst.at(0);
          results.push_back(cand);
          howmany.push_back(1);
          total++;
        }
        else
        {
          if (name.compare(vst.at(0)) == 0)
          {
            if (!results.empty())
            {
              bool found=false;
              for (int i=0; i<results.size(); ++i)
              {
                if(cand.compare(results[i]) == 0)
                {
                  found = true;
                  howmany[i]++;
                  break;
                }
              }
              if (!found)
              {
                results.push_back(cand);
                howmany.push_back(1);
              }
              ++total;
            }
            else
            {
              results.push_back(cand);
              howmany.push_back(1);
              ++total;
            }
          }
          else
          {
            ofstream unk;
            unk.open ("unknown.txt", fstream::out | fstream::app);
            unk<<name<<" Tot "<<total<<endl;
            cout<<name<<" Tot "<<total<<endl;
            for (size_t i=0; i<results.size(); ++i)
            {
              unk<<"\t"<<results[i]<<"\t"<<howmany[i]<<"\t";
              cout<<"\t"<<results[i]<<"\t"<<howmany[i]<<"\t";
            }
            unk<<endl<<endl;
            results.clear();
            howmany.clear();
            total=0;
            name = vst[0];
            results.push_back(cand);
            howmany.push_back(1);
            ++total;
            unk.close();
          }
        }
      }
      ofstream unk;
      unk.open ("unknown.txt", fstream::out | fstream::app);
      unk<<name<<" Tot "<<total<<endl;
      cout<<name<<" Tot "<<total<<endl;
      for (size_t i=0; i<results.size(); ++i)
      {
        unk<<"\t"<<results[i]<<"\t"<<howmany[i]<<"\t";
        cout<<"\t"<<results[i]<<"\t"<<howmany[i]<<"\t";
      }
      unk<<endl<<endl;
      cout<<endl<<endl;
      unk.close();
    }
  }
}
