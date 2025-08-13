/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Aaron Wilk 2025, All rights reserved.                     */
/*                                                                            */
/*    Module:     titanselect.hpp                                             */
/*    Author:     Aaron Wilk                                                  */
/*    Created:    25 July 2025                                                */
/*                                                                            */
/*    Revisions:  V0.1                                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef TITANSELECT_HPP
#define TITANSELECT_HPP

//TITANSELECT SETTINGS.
///@brief The number of rows the auton selector has.
#define SELECTOR_ROWS 3
///@brief The number of columns the auton selector has.
#define SELECTOR_COLS 3

//TITANSELECT API Macros

/// @brief Defines an auton and automatically adds it to the selector
/// @param name Name of the auton
/// @param routine of auton in curly braces {}.
/// Example - AUTON(BlueLeft, {MoveRobot();});
#define AUTON(name, routine) inline static ts::auton name(#name, []()routine)

/// @brief Defines an auton and automatically adds it to the selector
/// @param name Name of the auton
/// @param function cpp function that matches the void() function signature. Can be a lambda or raw function pointer
/// EXAMPLE AUTON_RAW(BlueRight, MyAutonFunction);
#define AUTON_RAW(name, function) inline static ts::auton name(#name, function)

/// @brief Initializes the selector and displays it on the screen.
#define SELECTOR_INIT() ts::selector::Get()->Create();

/// @brief Runs the selected auton.
#define SELECTOR_RUN_SELECTED() ts::selector::Get()->RunSelectedAuton()

/// @brief Runs the auton of the specified name.
/// @param auton_name Name of the auton to run.
#define SELECTOR_RUN_AUTON(auton_name) ts::selector::Get()->RunAuton(auton_name)

/// @brief Whether an auton is selected.
/// @return A boolean representing whether an auton is selected.
#define SELECTOR_IS_AUTON_SELECTED() ts::selector::Get()->IsAutonSelected()

/// @brief The name of the currently selected auton
/// @retval No_Auton No auton has been selected, actually returns "No Auton".
/// @retval any The name of the current selected auton.
#define SELECTOR_SELECTED_AUTON_NAME() ts::selector::Get()->SelectedAutonName()

/// @brief Sets the visibility of the selector. Useful for incorporating your own displays.
/// @param visible Visibility toggle the selector should be set too. false is hidden and true is visible.
#define SELECTOR_SET_VISIBILITY(visible) ts::selector::Get()->SetVisibility(visible)

/// @brief Get the active selector object
/// @retval selector The active selector object.
#define SELECTOR_GET_SELECTOR() ts::selector::Get()

/// @brief Destroys the selectors object in memory. Use with caution.
#define SELECTOR_DESTROY() ts::selector::Destroy()


//TODO: MAKE Add visual button border to make the selected auton more obvious



//IMPLEMENTATION BELOW! DO NOT CHANGE!

#include <functional>
#include <cstring>
#include <utility>
#include <fstream>
#include <filesystem>
#include "../liblvgl/lvgl.h"
#include "../pros/misc.hpp"

#define SELECTOR_HEIGHT 200
#define SELECTOR_WIDTH 475
#define SELECTOR_X_OFFSET 0
#define SELECTOR_Y_OFFSET 0
#define SELECTOR_NO_AUTON_TEXT "No Auton"
#define SELECTOR_INVALID_AUTON_TEXT "Invalid Auton"
#define SELECTOR_BUTTON_X 290
#define SELECTOR_BUTTON_Y 5
#define SELECTOR_BUTTON_WIDTH 180
#define SELECTOR_BUTTON_HEIGHT 30
#define SELECTOR_BUTTON_TEXT "Test Selected Auton"
#define SELECTOR_LABEL_TEXT "Selected: "
#define SELECTOR_LABEL_X 10
#define SELECTOR_LABEL_Y 6
#define SELECTOR_AUTON_FILE_PATH "/usd/LastSelectedAuton.txt"

#define STREQL(str, str2) strcmp(str, str2) == 0

namespace ts
{
    struct auton;

    class registry
    {
        public:
            inline static std::vector<auton*> autons;
            static void Register(auton* routine)
            {
                autons.push_back(routine);
            }
    };

    struct auton
    {
        const char* AutonName;
        const std::function<void()> FunctionRef;
        auton(const char* name, std::function<void()> func) : AutonName(name), FunctionRef(std::move(func))
        {
            registry::Register(this);
        }
    };

    class selector
    {
        inline static selector* instance;
        const char* aSelectedAuton;

        lv_obj_t* lButtonMatrix;
        lv_obj_t* lSelectedAutonLabel;
        lv_obj_t* lRunSelectedAutonButon;
        lv_obj_t* lRunSelectedAutonButtonLabel;

        selector() :
        aSelectedAuton(SELECTOR_NO_AUTON_TEXT),
        lButtonMatrix(nullptr),
        lSelectedAutonLabel(nullptr),
        lRunSelectedAutonButon(nullptr),
        lRunSelectedAutonButtonLabel(nullptr)
        {
            ReadSavedAuton();//Loads if an auton is present
        }

        static void SetObjectHidden(lv_obj_t* obj, bool hidden);
        static void Destroy(); //Destroys and deletes the current instance

        void WriteSavedAuton() const;
        void ReadSavedAuton();//Will also set aSelectedAuton
        void SetVisuallySelectedAuton(const char* name) const;
        ~selector(); //Explicit deconstructor for lvgl objects

        public:

        void Create();
        void RunSelectedAuton() const;
        void RunAuton(const char* name) const;
        void SetVisibility(bool visible) const;
        bool IsAutonSelected() const;
        const char* GetSelectedAutonName() const;
        static selector* Get();

        private:

        static void HandleEvents(lv_event_t * e);
    };

    inline void selector::SetObjectHidden(lv_obj_t *obj, bool hidden)
    {
        hidden ? lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN) : lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }

    inline void selector::Destroy()
    {
        delete instance;
        instance = nullptr;
    }

    inline void selector::WriteSavedAuton() const
    {
        std::ofstream file(SELECTOR_NO_AUTON_TEXT);
        if (!file.is_open()) return;
        file << aSelectedAuton;
        file.close();
    }

    inline void selector::ReadSavedAuton()
    {
        std::ifstream AutonFile(SELECTOR_AUTON_FILE_PATH);
        if (!AutonFile) return;
        std::string line;
        if (!std::getline(AutonFile, line)) return;
        //Check if auton exists compared to ones in the binary
        static const char* name = line.c_str();
        if (STREQL(name, SELECTOR_NO_AUTON_TEXT)) return;
        aSelectedAuton = SELECTOR_NO_AUTON_TEXT;
        for (auton* autonomous : registry::autons)
        {
            if (strcmp(autonomous->AutonName, name) == 0)
            {
                aSelectedAuton = name;
                break;
            }
        }
        AutonFile.close();
    }

    inline void selector::SetVisuallySelectedAuton(const char *name) const
    {
        //TODO implement
    }

    inline selector::~selector()
    {
        lv_obj_delete(lButtonMatrix);
        lv_obj_delete(lSelectedAutonLabel);
        lv_obj_delete(lRunSelectedAutonButon);
        lv_obj_delete(lRunSelectedAutonButtonLabel);
    }

    inline void selector::Create()
    {
        lv_obj_t * btnm = lv_buttonmatrix_create(lv_screen_active());
        static const char* btn_map[SELECTOR_ROWS * SELECTOR_COLS + SELECTOR_COLS + 1] = {};


        //Normally this would go rows->cols, but button matrix likes to be difficult
        short aIndex = 0; //Index for taking stuff out of the autons
        short rIndex = 0; //Index for the one dimensional array of button map
        std::vector<auton*> autons = registry::autons;
        for (int i = 0; i < SELECTOR_COLS; i++)
        {
            for (int j = 0; j < SELECTOR_ROWS; j++)
            {
                //Item will not be valid
                if (autons.size() > aIndex)
                {
                    if (autons[aIndex] == nullptr)
                    {
                        btn_map[rIndex] = SELECTOR_INVALID_AUTON_TEXT;
                    } else
                    {
                        btn_map[rIndex] = autons[aIndex]->AutonName;
                    }
                } else
                {
                    btn_map[rIndex] = SELECTOR_NO_AUTON_TEXT;
                }
                aIndex++;
                rIndex++;
            }
            btn_map[rIndex] = "\n";
            rIndex++;
        }
        btn_map[rIndex] = "";


        lv_buttonmatrix_set_map(btnm, btn_map);

        lv_obj_set_size(btnm, SELECTOR_WIDTH, SELECTOR_HEIGHT);
        lv_obj_align(btnm, LV_ALIGN_BOTTOM_MID, SELECTOR_X_OFFSET, SELECTOR_Y_OFFSET);

        lv_obj_add_event_cb(btnm, selector::HandleEvents, LV_EVENT_VALUE_CHANGED, nullptr);

        lButtonMatrix = btnm;


        lv_obj_t* btnau = lv_button_create(lv_screen_active());
        lv_obj_set_size(btnau, SELECTOR_BUTTON_WIDTH, SELECTOR_BUTTON_HEIGHT);
        lv_obj_set_pos(btnau, SELECTOR_BUTTON_X, SELECTOR_BUTTON_Y);
        lv_obj_t* btnlabel = lv_label_create(btnau);
        lv_label_set_text(btnlabel, SELECTOR_BUTTON_TEXT);
        lv_obj_align(btnlabel, LV_ALIGN_CENTER, 0, 0);

        lRunSelectedAutonButtonLabel = btnau;
        lRunSelectedAutonButtonLabel = btnlabel;

        std::string labelText = SELECTOR_LABEL_TEXT;
        labelText.append(SELECTOR_NO_AUTON_TEXT);
        lv_obj_t* label = lv_label_create(lv_screen_active());
        lv_label_set_text(label, labelText.c_str());
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
        lv_obj_set_pos(label, SELECTOR_LABEL_X, SELECTOR_LABEL_Y);
        lSelectedAutonLabel = label;

    }

    inline void selector::RunSelectedAuton() const
    {
        if (STREQL(aSelectedAuton, SELECTOR_NO_AUTON_TEXT))
        {
            return;
        }
        RunAuton(aSelectedAuton);
    }

    inline void selector::RunAuton(const char *name) const
    {
        if (STREQL(name, SELECTOR_NO_AUTON_TEXT)) return;
        for (auton* autonomous : registry::autons)
        {
            if (strcmp(autonomous->AutonName, name) == 0)
            {
                autonomous->FunctionRef();
                return;
            }
        }
    }

    inline void selector::SetVisibility(bool visible) const
    {
        SetObjectHidden(lButtonMatrix, visible);
        SetObjectHidden(lSelectedAutonLabel, visible);
        SetObjectHidden(lRunSelectedAutonButon, visible);
        SetObjectHidden(lRunSelectedAutonButtonLabel, visible);
    }

    inline bool selector::IsAutonSelected() const
    {
        return aSelectedAuton != nullptr && STREQL(aSelectedAuton, SELECTOR_NO_AUTON_TEXT);
    }

    inline const char * selector::GetSelectedAutonName() const
    {
        return aSelectedAuton;
    }

    inline selector * selector::Get()
    {
        //Lifetime heap variables do not need to be freed.
        if (!instance) instance = new selector;
        return instance;
    }

    inline void selector::HandleEvents(lv_event_t *e)
    {
        //There would, be a checking of event type here, but since it is only called on presses it does not matter.
        lv_obj_t * obj = lv_event_get_target_obj(e); // Get the button matrix object
        if (obj == Get()->lRunSelectedAutonButon)
        {
            auto master = pros::Controller(pros::E_CONTROLLER_MASTER);
            master.rumble("- - -");
            //Test if rumble waits.
            Get()->RunSelectedAuton();
        }
        else if (obj == Get()->lButtonMatrix)
        {
            uint32_t btn_id = lv_buttonmatrix_get_selected_button(obj); // Get the ID of the pressed/released button
            const char * btn_text = lv_buttonmatrix_get_button_text(obj, btn_id);
            Get()->aSelectedAuton = btn_text;
            std::string format = SELECTOR_LABEL_TEXT;
            format.append(Get()->aSelectedAuton);
            lv_label_set_text(Get()->lSelectedAutonLabel, format.c_str());
            Get()->WriteSavedAuton(); //Write the new selected auton
        }
    }
}

#endif //TITANSELECT_HPP
