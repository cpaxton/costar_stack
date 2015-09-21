#include "../include/utility.h"

//*
int main(int argc, char** argv)
{
    std::string num_rot(argv[1]);
    std::string filename("uniRot_txt/UniRot_"+num_rot+".txt");
    std::ifstream fp(filename.c_str(), std::ios::in);
    
    poseVec vec;
    if(fp.is_open())
    {
        int num, width;
        fp >> num >> width;
        for( int i = 0 ; i < num ; i++ )
        {
            Eigen::Matrix4f cur_rot;
            fp >> cur_rot(0, 0) >> cur_rot(0, 1) >> cur_rot(0, 2) >> cur_rot(1, 0) >> cur_rot(1, 1) >> cur_rot(1, 2)
                    >> cur_rot(2, 0) >> cur_rot(2, 1) >> cur_rot(2, 2);
            if( checkTrans(cur_rot) == true )
                vec.push_back(cur_rot);
        }
        fp.close();
        
        std::ostringstream ss;
        ss << vec.size();
        std::string filename_out("uniRot_txt/SemiRot_"+ss.str()+".txt");
        std::ofstream fp_out(filename_out.c_str(), std::ios::out);
        fp_out << vec.size() << " " << 9 << std::endl;
        for( int i = 0 ; i < vec.size() ; i++ )
            fp_out << vec[i](0, 0) << " " << vec[i](0, 1) << " "<< vec[i](0, 2) << " "<< vec[i](1, 0) << " "
                    << vec[i](1, 1) << " "<< vec[i](1, 2) << " " << vec[i](2, 0) << " " << vec[i](2, 1) << " " << vec[i](2, 2) << std::endl;
        fp_out.close();
    }  
    return 0;
}
//*/
