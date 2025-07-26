//
// Created by aaron on 7/25/2025.
//

#ifndef TITANSELECT_HPP
#define TITANSELECT_HPP

#include <functional>

#include "../liblvgl/lvgl.h"
#include "../pros/misc.hpp"
#include <cstring>
#include <utility>

#define SELECTOR ts::selector::Get()
#define SELECTOR_HEIGHT 200
#define SELECTOR_WIDTH 475
#define SELECTOR_ROWS 3
#define SELECTOR_COLS 3
#define SELECTOR_X_OFFSET 0
#define SELECTOR_Y_OFFSET -18
#define SELECTOR_NO_AUTON_TEXT "No Auton"
#define SELECTOR_INVALID_AUTON_TEXT "Invalid Auton"
#define SELECTOR_BUTTON_WIDTH 60
#define SELECTOR_BUTTON_HEIGHT 20
#define SELECTOR_BUTTON_TEXT "Test Selected Auton"
#define SELECTOR_LABEL_TEXT "Selected Auton: "

#define STREQL(str, str2) strcmp(str, str2) == 0

/// <summary>
/// Defines an auton
/// </summary>
/// <param name="name">Name of the auton</param>
/// <param name="routine">Code of the auton in brackets {}</param>
/// <returns>None</returns>
#define AUTON(name, routine) \
static ts::auton name(#name, []()routine); \

namespace ts
{
    struct auton;

    class registry
    {
    public:
        inline static std::vector<auton*> autons;
        inline static void Register(auton* routine)
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

    enum e_handle_callback
    {
        SELECTION_NONE = 0,
        SELECTION_NO_AUTON = 1,
        SELECTION_UNMATCHED = 2,
    };

    class selector
    {
        inline static selector* instance;
        const char* aSelectedAuton;
        std::function<void(ts::e_handle_callback)> fCustomErrorCallbackFunction;

        lv_buttonmatrix_t* lButtonMatrix;
        lv_label_t* lSelectedAutonLabel;
        lv_button_t* lRunSelectedAutonButon;
        lv_label_t* lRunSelectedAutonButtonLabel;

        FILE* fSelectedAutonFile;

        selector() :
        aSelectedAuton(SELECTOR_NO_AUTON_TEXT),
        lButtonMatrix(nullptr),
        lSelectedAutonLabel(nullptr),
        lRunSelectedAutonButon(nullptr),
        lRunSelectedAutonButtonLabel(nullptr),
        fCustomErrorCallbackFunction(nullptr),
        fSelectedAutonFile(nullptr)
        {
            fopen("/usd/LastSelectedAuton.txt", "r");
        }

        static void SetObjectHidden(lv_obj_t* obj, bool hidden);

        void WriteSavedAuton();
        void ReadSavedAuton();

        public:

        void RegisterCustomErrorCallback(std::function<void(ts::e_handle_callback)> callback);
        void Create();
        void RunSelectedAuton() const;
        void RunAuton(const char* name) const;

        //Called when the selector has an issue
        void HandleCallback(e_handle_callback callback) const;

        static selector* Get();
        static void HandleEvents(lv_event_t * e);
    };

    inline void selector::SetObjectHidden(lv_obj_t *obj, bool hidden)
    {
        hidden ? lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN) : lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }

    inline void selector::WriteSavedAuton()
    {
        //Check file validity
    }

    inline void selector::ReadSavedAuton()
    {
        FILE* file = fopen("/usd/LastSelectedAuton.txt", "r");
        //Check the file validity
    }

    inline void selector::RegisterCustomErrorCallback(std::function<void(ts::e_handle_callback)> callback)
    {
        fCustomErrorCallbackFunction = std::move(callback);
    }

    inline void selector::Create()
    {
        lv_obj_t * btnm = lv_buttonmatrix_create(lv_screen_active());
        static const char* btn_map[SELECTOR_ROWS * SELECTOR_COLS + SELECTOR_COLS] = {};

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

        lv_buttonmatrix_set_map(btnm, btn_map);

        lv_obj_set_size(btnm, SELECTOR_WIDTH, SELECTOR_HEIGHT);
        lv_obj_align(btnm, LV_ALIGN_CENTER, SELECTOR_X_OFFSET, SELECTOR_Y_OFFSET);

        lv_obj_add_event_cb(btnm, selector::HandleEvents, LV_EVENT_VALUE_CHANGED, 0);

        lButtonMatrix = (lv_buttonmatrix_t*)btnm;

        lv_obj_t* btnau = lv_button_create(lv_screen_active());
        lv_obj_set_size(btnau, SELECTOR_WIDTH, SELECTOR_HEIGHT);
        lv_obj_t* btnlabel = lv_label_create(btnau);
        lv_label_set_text(btnlabel, SELECTOR_BUTTON_TEXT);

        lRunSelectedAutonButtonLabel = (lv_label_t*)btnlabel;

        std::string labelText = SELECTOR_LABEL_TEXT;
        labelText.append(SELECTOR_NO_AUTON_TEXT);
        lv_obj_t* label = lv_label_create(lv_screen_active());
        lv_label_set_text(label, labelText.c_str());
        lSelectedAutonLabel = (lv_label_t*)label;
    }

    inline void selector::RunSelectedAuton() const
    {
        if (STREQL(aSelectedAuton, SELECTOR_NO_AUTON_TEXT))
        {
            HandleCallback(SELECTION_NO_AUTON);
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
        HandleCallback(SELECTION_UNMATCHED);
    }

    inline void selector::HandleCallback(e_handle_callback callback) const
    {
        if (fCustomErrorCallbackFunction) fCustomErrorCallbackFunction(callback);
    }

    inline selector * selector::Get()
    {
        //Lifetime heap variables do not need to be freed.
        if (!instance) instance = new selector;
        return instance;
    }

    inline void selector::HandleEvents(lv_event_t *e)
    {
        if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
        lv_obj_t * obj = lv_event_get_target_obj(e); // Get the button matrix object
        if (obj == (lv_obj_t*)Get()->lRunSelectedAutonButon)
        {
            auto master = pros::Controller(pros::E_CONTROLLER_MASTER);
            master.rumble("- - -");
            //Test if rumble waits.
            Get()->RunSelectedAuton();
        }
        else if (obj == (lv_obj_t*)Get()->lButtonMatrix)
        {
            uint32_t btn_id = lv_buttonmatrix_get_selected_button(obj); // Get the ID of the pressed/released button
            const char * btn_text = lv_buttonmatrix_get_button_text(obj, btn_id);
            Get()->aSelectedAuton = btn_text;
            std::string format = SELECTOR_LABEL_TEXT;
            format.append(Get()->aSelectedAuton);
            lv_label_set_text((lv_obj_t*)Get()->lSelectedAutonLabel, format.c_str());
        }
    }
}

#endif //TITANSELECT_HPP
