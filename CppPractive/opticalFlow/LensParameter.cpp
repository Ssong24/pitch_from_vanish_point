//
//  LensParameter.cpp
//  m_stitcher
//
//  Created by Song on 2022/04/12.
//

#include "LensParameter.hpp"


using namespace std;

namespace lens {
    int getLensType(LensType Ltype){
        switch (Ltype) {
            case R_NOLENS:
            case HH_NOLENS:
                return 0;
                break;
            case R_DAISO:
            case HH_DAISO:
                return 1;
                break;
            case R_YOUVR:
            case HH_YOUVR:
                return 2;
                break;
            case R_SECONDARY_WIDE:
            case HH_SECONDARY_WIDE:
                return 3;
                break;
            case R_OLLO:
            case HH_OLLO:
                return 4;
                break;
                
            default:
                return 0;
                break;
        }
    }

    std::string getString(int i){
        
        std::stringstream ss;
        ss << i;
        std::string out = ss.str();
        return out;
    }


    bool modelFound(string haystack, string needle){
        
        if (haystack.find(needle) != std::string::npos) {
            return true;
        }
        else{
            return false;
        }
    }

    void LensLookup(string CSVurl, LensType Ltype, int model_idx, float &HFoV, float &K1, float &K2, float &K3, float &PPX, float &PPY)
    {
        std::ifstream file(CSVurl);
        
        std::string line;
        std::string cell;
        std::string idx_s, model;
        
        std::string found_hfov = "-1";
        std::string found_k1 = "0";
        std::string found_k2 = "0";
        std::string found_k3 = "0";
        std::string found_ppx = "0";
        std::string found_ppy = "0";
        
        vector<string> params;
        
        int LensTypeIdx = getLensType(Ltype);
        
        int r=-2;
        while(getline(file,line))
        {
            r++;
            
            if (r==0) { //get default values
                std::stringstream  lineStream(line);
                
                getline(lineStream,idx_s,',');
                getline(lineStream,model,',');
                
                getline(lineStream,found_ppx,',');
                getline(lineStream,found_ppy,',');
                
                params.clear();
                while(getline(lineStream,cell,',')){
                    params.push_back(cell);
                }
                
                found_hfov = params[LensTypeIdx*num_of_params + lens_hfov];
                found_k1 = params[LensTypeIdx*num_of_params + lens_k1];
                found_k2 = params[LensTypeIdx*num_of_params + lens_k2];
                found_k3 = params[LensTypeIdx*num_of_params + lens_k3];
            }
            else if (r==model_idx) { //model is found
                
                std::stringstream  lineStream(line);
                
                getline(lineStream,idx_s,',');
                getline(lineStream,model,',');
                cout << "MODEL: " << model << "!!!" <<  endl;
                
                getline(lineStream,found_ppx,',');
                getline(lineStream,found_ppy,',');
                
                params.clear();
                while(getline(lineStream,cell,',')){
                    params.push_back(cell);
                }
                
                found_hfov = params[LensTypeIdx*num_of_params + lens_hfov];
                found_k1 = params[LensTypeIdx*num_of_params + lens_k1];
                found_k2 = params[LensTypeIdx*num_of_params + lens_k2];
                found_k3 = params[LensTypeIdx*num_of_params + lens_k3];
                
                break;
            }
        }
        
        HFoV = std::atof(found_hfov.c_str());
        K1 = std::atof(found_k1.c_str());
        K2 = std::atof(found_k2.c_str());
        K3 = std::atof(found_k3.c_str());
        PPX = std::atoi(found_ppx.c_str());
        PPY = std::atoi(found_ppy.c_str());
    }

    void LensLookupCSV(string CSVcontent, LensType Ltype, string manf_tofind, string model_tofind, float &HFoV, float &K1, float &K2, float &K3)
    {
        std::stringstream  fullStream(CSVcontent);
        
        std::string line;
        std::string manf_info, model_info;
        std::string cell;
        
        std::string found_hfov = "-1";
        std::string found_k1 = "0";
        std::string found_k2 = "0";
        std::string found_k3 = "0";
        
        vector<string> params;
        
        int LensTypeIdx = getLensType(Ltype);
        std::transform(manf_tofind.begin(), manf_tofind.end(), manf_tofind.begin(), ::tolower);
        std::transform(model_tofind.begin(), model_tofind.end(), model_tofind.begin(), ::tolower);
        
        while(getline(fullStream,line))
        {
            std::stringstream  lineStream(line);
            
            getline(lineStream,manf_info,',');
            std::transform(manf_info.begin(), manf_info.end(), manf_info.begin(), ::tolower);
            
            if (manf_info == "unknown") //---Get default value for unknown Manufacturer
            {
                getline(lineStream,model_info,',');
                
                while(getline(lineStream,cell,',')){
                    params.push_back(cell);
                }
                found_hfov = params[LensTypeIdx*num_of_params + lens_hfov];
                found_k1 = params[LensTypeIdx*num_of_params + lens_k1];
                found_k2 = params[LensTypeIdx*num_of_params + lens_k2];
                found_k3 = params[LensTypeIdx*num_of_params + lens_k3];
            }
            else if (manf_info == manf_tofind) //---Manufacturer found
            {
                getline(lineStream,model_info,',');
                std::transform(model_info.begin(), model_info.end(), model_info.begin(), ::tolower);
                
                if(model_info == "other")  //---Get default value for this manufacturer
                {
                    params.clear();
                    while(getline(lineStream,cell,',')){
                        params.push_back(cell);
                    }
                    found_hfov = params[LensTypeIdx*num_of_params + lens_hfov];
                    found_k1 = params[LensTypeIdx*num_of_params + lens_k1];
                    found_k2 = params[LensTypeIdx*num_of_params + lens_k2];
                    found_k3 = params[LensTypeIdx*num_of_params + lens_k3];
                }
                else if(modelFound(model_info, model_tofind))  //---Device's model is found
                {
                    params.clear();
                    while(getline(lineStream,cell,',')){
                        params.push_back(cell);
                    }
                    found_hfov = params[LensTypeIdx*num_of_params + lens_hfov];
                    found_k1 = params[LensTypeIdx*num_of_params + lens_k1];
                    found_k2 = params[LensTypeIdx*num_of_params + lens_k2];
                    found_k3 = params[LensTypeIdx*num_of_params + lens_k3];

                    break;
                }
            }
        }
        
        HFoV = std::atof(found_hfov.c_str());
        K1 = std::atof(found_k1.c_str());
        K2 = std::atof(found_k2.c_str());
        K3 = std::atof(found_k3.c_str());
    }

    void LensLookupJSON(std::string json_src, LensType Ltype, int& status_code, float &HFoV, float &K1, float &K2, float &K3, float &Ppx, float &Ppy)
    {
        int LensTypeIdx = getLensType(Ltype);
        
        Json::Reader reader;
        Json::Value root;
        Json::Value params;
        
        // Check if input is json file url or json string content
        bool parsingSuccessful = reader.parse( json_src.c_str(), root );
        if (!parsingSuccessful)
        {
            ifstream infile (json_src.c_str(), std::ifstream::binary);
            parsingSuccessful = reader.parse( infile, root );
        }
        
        
        if (parsingSuccessful)
        {
            params = root[getString(LensTypeIdx)];
            
            status_code = params.get("status", 0).asInt();
            HFoV = params.get("HFoV", -1).asFloat();
            K1 = params.get("k1", 0.0).asFloat();
            K2 = params.get("k2", 0.0).asFloat();
            K3 = params.get("k3", 0.0).asFloat();
            Ppx = params.get("ppx", 0.0).asFloat();
            Ppy = params.get("ppy", 0.0).asFloat();
        }
        else {
            HFoV = -1;
        }
    }


    bool readParamFromJson(string jsonFilePath, int lensType, int deviceModelNum,  struct cameraParam &camP) {
        
        string inputLensType = to_string(lensType);

        Json::Reader reader;
        Json::Value root;
        
        
        // Check if input is json file url or json string content
        bool parsingSuccessful = reader.parse( jsonFilePath.c_str(), root );
        
        camP.Ppx = 0.0f;
        camP.Ppy = 0.0f;


        if (parsingSuccessful) {
            Json::Value parameter = root;
            Json::ValueIterator it = parameter.begin();
 
            while (it != parameter.end()) {

                int deviceNum = (*it)["deviceNumber"].asInt();

                if ((int)deviceNum == (int)deviceModelNum)
                {

                    Json::Value rdata = (*it)["data"][inputLensType];
                    camP.HFov = rdata["HFoV"].asFloat();
                    camP.lensType = lensType;
                    camP.k1 = rdata["k1"].asFloat();
                    camP.k2 = rdata["k2"].asFloat();
                    camP.k3 = rdata["k3"].asFloat();

                }
                it++;
            }
        }
        else {
            printf("Parsing will be read as binary mode\n");
            ifstream infile (jsonFilePath.c_str(), std::ifstream::binary);
            
            reader.parse( infile, root );

            Json::Value parameter = root;            
            Json::ValueIterator it = parameter.begin();
            
            while (it != parameter.end()) {

                int deviceNum = (*it)["deviceNumber"].asInt();
                if (deviceNum == deviceModelNum) 
                {
                    Json::Value rdata = (*it)["data"][inputLensType];
                    
                    camP.HFov = rdata["HFoV"].asFloat();
                    camP.lensType = lensType;
                    camP.k1 = rdata["k1"].asFloat();
                    camP.k2 = rdata["k2"].asFloat();
                    camP.k3 = rdata["k3"].asFloat();
//                    cout << "HFov, k3: " << camP.HFov << ", " << camP.k3 << endl;
                    
                }
                it++;
            }
            
        } 
        
        

        
        return true;
    }

    bool readParamFromCSV(const char * csvURL, int model, lens::LensType activeLens, struct cameraParam& camP)  { //  float& HFoV, float& k1, float& k2, float& k3 ) {
        if (csvURL != NULL && model >= 0) {
            float hfovLookup, k1Lookup, k2Lookup, k3Lookup, ppxLookup, ppyLookup;
            
            lens::LensLookup(csvURL, activeLens, model, hfovLookup, k1Lookup, k2Lookup, k3Lookup, ppxLookup, ppyLookup);
            
            if(hfovLookup != -1) {
                camP.HFov = hfovLookup;
                camP.k1 = k1Lookup;
                camP.k2 = k2Lookup;
                camP.k3 = k3Lookup;
                
                cout<<"hfov, k1, k2, k3, ppx, ppy: " << hfovLookup<<"|"<<k1Lookup<<"|"<<k2Lookup<<"|"<<k3Lookup<<"|"<<ppxLookup<<"|"<<ppyLookup<<endl;
                return EXIT_SUCCESS;
                
            }
            else {  // Even though no CSV content, program goes on
                cout<<"CSV content is empty or model not found!"<<endl;
                
            }
        }
        return EXIT_FAILURE;
        
    }
}



string set_calibData(int lens_type, float pitch, float HFov, float k1, float k2, float k3) {
    
    string calibData = "";
    
    calibData += "{\"lens_type\":" + to_string(lens_type);
    calibData += ",\"pitch\":" + to_string(pitch);
    calibData += ",\"HFoV\":" + to_string(HFov);
    calibData += ",\"k1\":" + to_string(k1);
    calibData += ",\"k2\":" + to_string(k2);
    calibData += ",\"k3\":" + to_string(k3);
    calibData += ",\"ppx\":0";
    calibData += ",\"ppy\":0";
    calibData += "}";
    
    return calibData;
}

void set_yrp_angle_ucd(float upYRP, float centerYRP, float downYRP, float yrpList[]) {
    yrpList[0] = upYRP;
    yrpList[1] = centerYRP;
    yrpList[2] = downYRP;
}

void save_userLens_data(int userLens, string& _srcDir, string & calibData, string& imgExt, Data_Stitch_Mode & _dsm, float tiltList[], float rollList[]) {
    string rootDir, lensDir;
    rootDir = "/Users/3i-21-331/workspace/stitching/mobile_stitching/dataset";
    float phoneTilt;
    
    /* Wide angle lens */
    if ( userLens == 3 ) {
        rootDir += "/Pivo/galaxy_s20_ultra";
        lensDir = "/3_wide";
    
        imgExt = ".jpg";
        _dsm.horizontal_only = true;
        
        calibData = set_calibData(3, 0.0, 78.0, 0.15, -1.0, 0.0);
    }

   else if (userLens == 5) {
       rootDir += "/PivoX_vertical/iphone_11_pro/";
       lensDir = "/3_wide";

       imgExt = ".JPG";
       _dsm.vertical_only = true;

       calibData = set_calibData(3, 0.0, 84.0, 0.0, -0.3, 0.0);
   }

    else if (userLens == 7) {
        rootDir += "/PivoX_full/iphone_pro_12";
        lensDir = "/Widelens_25deg";

        imgExt = ".JPG";
        _dsm.hv_horizontal = true;

        phoneTilt = 0.4;
        calibData = set_calibData(3, phoneTilt, 86, 0.02, -0.05, 0.03);
        set_yrp_angle_ucd(29.0, phoneTilt, -25, tiltList);
//        set_yrp_angle_ucd(0.3, 0.5, 0.8, rollList);

    }

//    else if (userLens == 9) {
//        rootDir += "/PivoX_full/motorola_g100";
//        lensDir = "/WideLens_25deg";
//
//        imgExt = ".jpg";
//        _dsm.hv_horizontal = true;
//
//        phoneTilt = 0.2;
//        calibData = set_calibData(3, phoneTilt, 81.42, 0.000751894, -0.0136778, 0.0168824);
//        set_yrp_angle_ucd(20.0, phoneTilt, -20.0, tiltList);
//    }
    
    else if (userLens == 11) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/correct_capture_1";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 1.6; // 1.6;  2.1(estimated) // 0.2;
        calibData = set_calibData(3, phoneTilt, 87.5,  0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
        set_yrp_angle_ucd(26, phoneTilt, -19.0, tiltList);  // 26, -19.0  ( 25, -2.1 -- estimated)
//        set_yrp_angle_ucd(-1.0, -0.7, -0.5, rollList);
    }
    
    else if (userLens == 13) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/correct_capture_2";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 0.5; // 1.5
        calibData = set_calibData(3, phoneTilt, 70,  0.004, -0.014, 0); // 87.5 0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
        set_yrp_angle_ucd(26, phoneTilt, -20, tiltList);
    }
    
    else if (userLens == 15) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/pitch_positive";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 8; //7;
        calibData = set_calibData(3, phoneTilt, 87.5,  0.004, -0.014, 0); // 0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
        set_yrp_angle_ucd(31, phoneTilt, -18, tiltList); // 30, phoneTilt, -18
//        set_yrp_angle_ucd(-2.0, -1.8, -1.3, rollList);
    }
    
    else if (userLens == 17) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/roll_negative";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 2.1;
        calibData = set_calibData(3, phoneTilt, 87.5, 0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
        set_yrp_angle_ucd(27.5, phoneTilt, -20, tiltList);  // 28
//        set_yrp_angle_ucd(-6.8, -6.3, -5.9, rollList);
    }
    
    else if (userLens == 19) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/roll_modified";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 1.6;
        calibData = set_calibData(3, phoneTilt, 78, 0.004, -0.014, 0); // 87.5  0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
        set_yrp_angle_ucd(26, phoneTilt, -19., tiltList);  // 26, -19
    }
    
//    else if (userLens == 21) {
//        rootDir += "/PivoX_full/galaxy_zflip3";
//        lensDir = "/WideLens_20deg/roll_modified_2";
//
//        imgExt = ".jpg";
//        _dsm.hv_horizontal = true;
//
//        phoneTilt = 1.9;
//        calibData = set_calibData(3, phoneTilt, 87.5, 0.15, -1, 0); //87.18, -0.01, 0.023, -0.015);
//        set_yrp_angle_ucd(26, phoneTilt, -20, tiltList);
//    }
    
    else if (userLens == 23) {
        rootDir += "/PivoX_full/galaxy_s22_ultra";
        lensDir = "/WideLens_20deg/1";
        
        imgExt = ".jpg";
        
        _dsm.hv_horizontal = true;
        
        phoneTilt = -2.3;
        calibData = set_calibData(3, phoneTilt, 78, 0.15, -1, 0);
        set_yrp_angle_ucd(21.5, phoneTilt, -23, tiltList);
        
    }
    
    else if (userLens == 25) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/FHD/1";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = -1.1;
        calibData = set_calibData(3, phoneTilt, 87.5, 0.001, -0.015, 0.000); // 0.15, -1, 0);
        
        set_yrp_angle_ucd(24, phoneTilt, -19, tiltList);
    }
    
    else if (userLens == 27) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/WideLens_20deg/FHD/2_wrong_order";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = -0.7;
        calibData = set_calibData(3, phoneTilt, 87.5, 0.001, -0.015, 0.000); // 0.15, -1, 0);  // 71.
        
        set_yrp_angle_ucd(24.4, phoneTilt, -21.0, tiltList);
    }
    
    else if (userLens == 29) {
        rootDir += "/PivoX_full/galaxy_s22_ultra";
        lensDir = "/WideLens_20deg/2";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = -0.8;
        calibData = set_calibData(3, phoneTilt, 87.5, 0.15, -1, 0);
        set_yrp_angle_ucd(23, phoneTilt, -22.2, tiltList);
    }

    else if (userLens == 31) {
        rootDir += "/PivoX_full/synthetic_data/1";
        lensDir = "";
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.0;
        calibData = set_calibData(3, phoneTilt, 80);
        set_yrp_angle_ucd(30, phoneTilt, -30, tiltList);
    }
    
    else if (userLens == 33) {
        rootDir += "/PivoX_full/synthetic_data/2";
        lensDir = "";
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.0;
        calibData = set_calibData(3, phoneTilt, 80);
        set_yrp_angle_ucd(30, phoneTilt, -30, tiltList);
    }


    
//    /* Normal angle Lens */
//    else if (userLens == 0) {
//        rootDir += "/Pivo/iphone_pro_11_max";
//        lensDir = "/0_nolens";
//
//        imgExt = ".jpeg";
//        _dsm.horizontal_only = true;
//
//        calibData = set_calibData(0, 0.0, 40.0);
//    }
//
//    else if (userLens == 4) {
//        rootDir += "/pivoX_vertical/iphone_11_pro";
//        lensDir = "/0_nolens";
//
//        imgExt = ".JPG";
//        _dsm.vertical_only = true;
//
//        calibData = set_calibData(0, 0.0, 42.0);
//    }
//
//    else if (userLens == 6) {
//        rootDir += "/PivoX_full/iphone_pro_12";
//        lensDir = "/Nolens_15deg";
//
//        imgExt = ".JPG";
//        _dsm.hv_horizontal = true;
//
//        calibData = set_calibData(0, 0.2, 40.0);
//        set_yrp_angle_ucd(25.0, 0.2, -22, tiltList);
//    }
    
    else if (userLens == 8) {  // hv_mode = trueW
        rootDir += "/PivoX_full/iphone_pro_12";
        lensDir = "/Nolens_25deg";// "/0_nolens";

        imgExt = ".JPG";
        _dsm.hv_horizontal = true;

        calibData = set_calibData(0, 0.0, 52.0);
        set_yrp_angle_ucd(28.5, 0.2, -26.5, tiltList);
    }

//    else if (userLens == 10) {
//        rootDir += "/PivoX_full/iphone_pro_12";
//        lensDir = "/Nolens_25deg_c";// "/0_nolens";
//
//        imgExt = ".JPG";
//        _dsm.horizontal_only = true;
//
//        calibData = set_calibData(0, 0.5, 40.79, -0.012, 0.0, -0.04);
//    }
//


    else if (userLens == 22) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/NoLens_20deg/3by4/correct_capture_1";  

        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 1.2;
        calibData = set_calibData(0, phoneTilt, 34); // 51  34
        set_yrp_angle_ucd(26, phoneTilt, -20.0, tiltList);  // 26, -20.0
    }

    else if (userLens == 24) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/NoLens_20deg/3by4/correct_capture_2";

        imgExt = ".jpg";
        _dsm.hv_horizontal = true;

        calibData = set_calibData(0, 0.0, 34);
        set_yrp_angle_ucd(20.0, 2.0, -20.0, tiltList);
    }


   else if (userLens == 28) {
       rootDir += "/PivoX_full/galaxy_zflip3";
       lensDir = "/NoLens_20deg/3by4/roll_positive";

       imgExt = ".jpg";
       _dsm.hv_horizontal = true;
       phoneTilt = 0.8;
       calibData = set_calibData(0, phoneTilt, 30);
       set_yrp_angle_ucd(24.5, phoneTilt, -22.8, tiltList);
   }
    
    else if (userLens == 30) {
        rootDir += "/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/HD";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 2.3;
        calibData = set_calibData(0, phoneTilt, 45, 0.168, -0.48, 0); // hfov =50
        set_yrp_angle_ucd(26.0, phoneTilt, -19, tiltList);
    }
    
    else if (userLens == 32) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/NoLens_20deg/FHD";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 0.7;
        calibData = set_calibData(0, phoneTilt, 40);   // 40
        set_yrp_angle_ucd(24.8, phoneTilt, -23.5, tiltList);
        
    }
    
    else if (userLens == 34) {
        rootDir += "/PivoX_full/galaxy_zflip3";
        lensDir = "/NoLens_20deg/HD/1";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        
        phoneTilt = 0.; // -1.6
        calibData = set_calibData(0, phoneTilt, 31);
        set_yrp_angle_ucd(24, phoneTilt, -21, tiltList);
    }
    
    else if (userLens == 36) {
        rootDir +="/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/1704x3648";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.37;
        calibData = set_calibData(0, phoneTilt, 45);
        set_yrp_angle_ucd(28.7, phoneTilt, -15.6, tiltList);
    }
    
    else if (userLens == 38) {
        rootDir +="/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/2052x3648";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 1.1;
        calibData = set_calibData(0, phoneTilt, 45, 0.168, -0.48, 0);
        set_yrp_angle_ucd(24.7, phoneTilt, -21.8, tiltList);
    }
    
    else if (userLens == 40) {
        rootDir +="/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/2160x3840";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.3;
        calibData = set_calibData(0, phoneTilt, 45); //0.149, -0.299, 0.004);
        set_yrp_angle_ucd(24.6, phoneTilt, -21.7, tiltList);
    }
    
    else if (userLens == 42) {
        rootDir +="/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/1";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.3;
        calibData = set_calibData(0, phoneTilt, 45, 0.149, -0.299, 0.004);
        set_yrp_angle_ucd(24.6, phoneTilt, -21.7, tiltList);
    }
    
    else if (userLens == 44) {
        rootDir +="/PivoX_full/galaxy_s22_ultra";
        lensDir = "/NoLens_20deg/2";
        
        imgExt = ".jpg";
        _dsm.hv_horizontal = true;
        phoneTilt = 0.3;
        calibData = set_calibData(0, phoneTilt, 45, 0.149, -0.299, 0.004);
        set_yrp_angle_ucd(24.6, phoneTilt, -21.7, tiltList);
    }
    
    
    
    printf("[%2d] lensDir: %s\n", userLens, lensDir.c_str());
    _srcDir = rootDir + lensDir;
}
