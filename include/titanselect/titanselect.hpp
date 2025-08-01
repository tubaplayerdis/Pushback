//
// Created by aaron on 7/25/2025.
//

#ifndef TITANSELECT_HPP
#define TITANSELECT_HPP

#include <functional>
#include <cstring>
#include <utility>
#include <fstream>
#include <filesystem>

#include "../liblvgl/lvgl.h"
#include "../pros/misc.hpp"

//Change these to determine the number of rows and columns on the titan select selector
//----------------------------
#define SELECTOR_ROWS 3
#define SELECTOR_COLS 3
//----------------------------

#define SELECTOR ts::selector::Get()
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

/// <summary>
/// Defines an auton
/// </summary>
/// <param name="name">Name of the auton</param>
/// <param name="routine">Code of the auton in brackets {}</param>
/// <returns>None</returns>
#define AUTON(name, routine) \
inline static ts::auton name(#name, []()routine); \

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

    class page
    {
        bool bIsEnabled;
        const char* cName;

        protected:

        explicit page(bool bEnableByDefault, const char* name) : bIsEnabled(bEnableByDefault), cName(name)
        {
            //TODO: Add this page to the page selector
        }
        virtual void OnSelected_Implementation() = 0;
        virtual void OnUnselected_Implementation() = 0;
        public:
        virtual ~page() = default;

        virtual void Create() = 0;
        void Select();
        void Unselect();
    };

    inline void page::Select()
    {
        //TODO: Make the page active with the page selector which still has to be done
        bIsEnabled = true;
        OnSelected_Implementation();
    }

    inline void page::Unselect()
    {
        bIsEnabled = false;
        OnUnselected_Implementation();
    }

    class selector : public page
    {
        inline static selector* instance;
        const char* aSelectedAuton;

        lv_obj_t* lButtonMatrix;
        lv_obj_t* lSelectedAutonLabel;
        lv_obj_t* lRunSelectedAutonButon;
        lv_obj_t* lRunSelectedAutonButtonLabel;

        FILE* fSelectedAutonFile;

        selector() :
        page(true, "selector"),
        aSelectedAuton(SELECTOR_NO_AUTON_TEXT),
        lButtonMatrix(nullptr),
        lSelectedAutonLabel(nullptr),
        lRunSelectedAutonButon(nullptr),
        lRunSelectedAutonButtonLabel(nullptr),
        fSelectedAutonFile(nullptr)
        {
            ReadSavedAuton();//Loads if an auton is present
        }

        static void SetObjectHidden(lv_obj_t* obj, bool hidden);

        //Saves current auton
        void WriteSavedAuton() const;
        //Reads last saved auton
        void ReadSavedAuton();

        protected:
        void OnSelected_Implementation() override;
        void OnUnselected_Implementation() override;

        public:

        void Create() override;
        void RunSelectedAuton() const;
        void RunAuton(const char* name) const;
        [[nodiscard]] bool IsAutonSelected() const;
        [[nodiscard]] const char* GetSelectedAutonName() const;

        static selector* Get();
        private:
        static void HandleEvents(lv_event_t * e);
    };

    inline void selector::SetObjectHidden(lv_obj_t *obj, bool hidden)
    {
        hidden ? lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN) : lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }

    inline void selector::WriteSavedAuton() const
    {
        std::ofstream file(SELECTOR_NO_AUTON_TEXT);
        if (!file.is_open()) return;
        file << aSelectedAuton;
        //Check file validity
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
        for (auton* autonomous : registry::autons)
        {
            if (strcmp(autonomous->AutonName, name) == 0)
            {
                aSelectedAuton = name;
                return;
            }
        }


        //Check the file validity
    }

    inline void selector::OnSelected_Implementation()
    {
        //TODO: Toggle the visibility of all objects
    }

    inline void selector::OnUnselected_Implementation()
    {
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
        WriteSavedAuton();
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
        }
    }
}

#endif //TITANSELECT_HPP
