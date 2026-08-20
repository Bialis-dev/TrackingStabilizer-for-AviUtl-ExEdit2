//----------------------------------------------------------------------------------
//  TrackingStabilizer for AviUtl ExEdit2
//
//  「モーショントラッキング(MK-II Plus)」等のトラッキングプラグインが出力した
//  図形オブジェクト(標準描画あり)の座標を毎フレーム読み取り、
//  基準フレームからの移動量の符号を反転して現在のオブジェクトへ加算することで
//  手ブレ補正(スタビライズ)を行うフィルタ効果プラグインです。
//
//  使い方の概要:
//   1. モーショントラッキングプラグインで映像のブレを追従する図形オブジェクトを
//      作成する(「部分フィルタとして挿入」はOFF、「Invert Position」もOFFのまま
//      = 素直にブレと同じ方向へ動く図形を作る)
//   2. 補正したい映像オブジェクトのフィルタ効果に本プラグインを追加する
//   3. 「追跡レイヤー1」に手順1の図形があるレイヤー番号(タイムライン表示の番号)を指定する
//   4. 「基準フレーム」にブレが無い基準となるフレーム番号(通常は先頭フレーム)を指定する
//   5. (任意) 回転・拡縮ブレも補正したい場合は、別の特徴点をもう1つ追跡して
//      図形を挿入し、「追跡レイヤー2」に指定したうえで「回転を補正する」/
//      「拡縮を補正する」をON にする
//
//  仕組み(平行移動):
//   get_output_image_param() を使って追跡レイヤーの図形オブジェクトの
//   「現在フレームの座標」と「基準フレームの座標」を取得し、その差(=ブレ量)の
//   符号を反転した値を video->param->x / y (対象オブジェクトの相対オフセット) に
//   加算します。
//
//  仕組み(回転、v1.03):
//   1点の座標だけでは「回転した」のか「平行移動した」のか区別できないため、
//   回転ブレの補正には2つの追跡点(追跡レイヤー1・追跡レイヤー2)を使います。
//   2点を結ぶ直線の角度を毎フレーム計算し、基準フレームでの角度との差分
//   (=回転ブレ量)の符号を反転した値を video->param->rz (Z軸回転、度単位) に
//   加算します。
//    d_angle = atan2(点2.y - 点1.y, 点2.x - 点1.x)の現在フレームと基準フレームの差
//    video->param->rz += -d_angle (度単位に変換)
//
//  仕組み(拡縮、v1.04):
//   同じ2つの追跡点を使い、今度は角度ではなく「2点間の距離」の変化を見ます。
//   現在フレームでの2点間距離が基準フレームより離れていれば「拡大して見えて
//   いる」とみなし、その比率の逆数を video->param->sx / sy (拡大率、1.0=等倍)
//   に掛け合わせることで、基準フレームの見た目の大きさに近づけます。
//    ratio = 基準フレームの2点間距離 / 現在フレームの2点間距離
//    video->param->sx *= ratio; video->param->sy *= ratio;
//
//  安全対策 (v1.01):
//   「追跡レイヤー」に図形以外のオブジェクト(音声、空、テキストのみ等)が
//   置かれていると、AviUtl2本体側の座標取得処理が失敗/例外を投げることがあり、
//   何もガードしないとアプリごと落ちて編集不能になる事故につながります。
//   そのため以下の二重の対策をしています。
//    (a) 呼び出し前に可能な限りポインタ・状態のチェックを行い、怪しい場合は
//        何もせず安全にreturnする。
//    (b) それでも万一AviUtl2側で例外(C++例外、アクセス違反等の構造化例外の
//        どちらでも)が発生した場合に備え、実処理を行う関数呼び出し全体を
//        __try/__except (SEH) で囲み、CMakeLists.txt側で /EHa (C++例外も
//        構造化例外として捕捉出来る例外処理モデル) を指定しています。
//        (__try/__exceptとC++のtry/catchはコンパイラの制約上、同一関数内で
//        同時に使えない場合があるため、実処理は別関数(try_stabilize)に分離
//        しています)
//
//  ノイズ対策 (v1.02):
//   トラッキング解析結果には、フレームごとに数ピクセル程度の細かいノイズ
//   (検出誤差によるガタつき)が乗ることがあります。60fps編集のまま等倍で
//   使う分にはほぼ気になりませんが、低いフレームレートで書き出すと1フレーム
//   あたりの表示時間が伸びるぶん、同じノイズが目立ちやすく「小刻みにガクガク
//   する」ように見える原因になります。
//   これを軽減するため、追跡座標を使う前に前後数フレームぶんの移動平均
//   (簡易ローパスフィルタ)をかける「平滑化(フレーム)」パラメータを追加しました。
//----------------------------------------------------------------------------------
#include <windows.h>
#include <algorithm>
#include <cmath>

#include "filter2.h"
#include "logger2.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool func_proc_video(FILTER_PROC_VIDEO* video);

//---------------------------------------------------------------------
//  ログ出力 (任意。取得出来ない場合はログを出さないだけで動作に支障なし)
//---------------------------------------------------------------------
static LOG_HANDLE* g_logger = nullptr;
static bool g_warned_no_track_object = false; // 同じ警告の連続出力を抑制するためのフラグ

EXTERN_C __declspec(dllexport) void InitializeLogger(LOG_HANDLE* logger) {
    g_logger = logger;
}

static void log_warn_once(LPCWSTR message) {
    if (g_warned_no_track_object) return; // フレーム毎に呼ばれるためログの連投を防止
    g_warned_no_track_object = true;
    if (g_logger && g_logger->warn) {
        g_logger->warn(g_logger, message);
    }
}

//---------------------------------------------------------------------
//  フィルタ設定項目定義
//---------------------------------------------------------------------
auto group_main    = FILTER_ITEM_GROUP(L"トラッキング手ブレ補正");
auto track_layer1  = FILTER_ITEM_TRACK(L"追跡レイヤー1",   1.0,   1.0, 200.0, 1.0);
auto base_frame    = FILTER_ITEM_TRACK(L"基準フレーム",    0.0,   0.0, 100000.0, 1.0);
auto strength      = FILTER_ITEM_TRACK(L"補正強度(%)",   100.0,   0.0, 200.0, 1.0);
auto smoothing     = FILTER_ITEM_TRACK(L"平滑化(フレーム)",  9.0,   1.0, 121.0, 2.0);
auto fix_x         = FILTER_ITEM_CHECK(L"X軸を補正する", true);
auto fix_y         = FILTER_ITEM_CHECK(L"Y軸を補正する", true);
auto invert        = FILTER_ITEM_CHECK(L"移動量を反転して適用する", true);

auto group_rotate   = FILTER_ITEM_GROUP(L"回転・拡縮ブレ補正(任意)");
auto track_layer2   = FILTER_ITEM_TRACK(L"追跡レイヤー2", 2.0, 1.0, 200.0, 1.0);
auto fix_rotate     = FILTER_ITEM_CHECK(L"回転を補正する", false);
auto rotate_strength = FILTER_ITEM_TRACK(L"回転補正強度(%)", 100.0, 0.0, 200.0, 1.0);
auto rotate_invert  = FILTER_ITEM_CHECK(L"回転方向を反転する", false);
auto fix_scale      = FILTER_ITEM_CHECK(L"拡縮を補正する", false);
auto scale_strength = FILTER_ITEM_TRACK(L"拡縮補正強度(%)", 100.0, 0.0, 200.0, 1.0);
auto scale_invert   = FILTER_ITEM_CHECK(L"拡縮方向を反転する", false);

auto group_end     = FILTER_ITEM_GROUP(L"");

void* items[] = {
    &group_main,
    &track_layer1,
    &base_frame,
    &strength,
    &smoothing,
    &fix_x,
    &fix_y,
    &invert,
    &group_rotate,
    &track_layer2,
    &fix_rotate,
    &rotate_strength,
    &rotate_invert,
    &fix_scale,
    &scale_strength,
    &scale_invert,
    &group_end,
    nullptr
};

//---------------------------------------------------------------------
//  フィルタプラグイン構造体定義
//---------------------------------------------------------------------
FILTER_PLUGIN_TABLE filter_plugin_table = {
    FILTER_PLUGIN_TABLE::FLAG_VIDEO | FILTER_PLUGIN_TABLE::FLAG_FILTER,   // 画像フィルタ + フィルタオブジェクト対応
    L"トラッキング手ブレ補正",                                             // プラグインの名前
    L"手ブレ補正",                                                         // ラベルの初期値
    L"TrackingStabilizer version 1.04 - motion tracking based stabilizer",// プラグインの情報
    items,                                                                 // 設定項目
    func_proc_video,                                                      // 画像フィルタ処理関数
};

//---------------------------------------------------------------------
//  プラグインDLL初期化関数
//---------------------------------------------------------------------
EXTERN_C __declspec(dllexport) bool InitializePlugin(DWORD version) {
    return true;
}

//---------------------------------------------------------------------
//  プラグインDLL解放関数
//---------------------------------------------------------------------
EXTERN_C __declspec(dllexport) void UninitializePlugin() {
}

//---------------------------------------------------------------------
//  フィルタ構造体のポインタを渡す関数
//---------------------------------------------------------------------
EXTERN_C __declspec(dllexport) FILTER_PLUGIN_TABLE* GetFilterPluginTable(void) {
    return &filter_plugin_table;
}

//---------------------------------------------------------------------
//  center_offset(現在時間からのオフセット秒)を中心に、前後 window フレーム分の
//  座標を平均して返す(簡易移動平均ローパスフィルタ)。
//  window <= 1 の場合は平均化せず単一サンプルを返す。
//  一部のフレームで取得に失敗しても、取得できたフレームだけで平均する。
//---------------------------------------------------------------------
static bool get_smoothed_param(FILTER_PROC_VIDEO* video, OBJECT_HANDLE obj,
                                double center_offset, int window, double fps,
                                OBJECT_IMAGE_PARAM& out) {
    if (window <= 1 || fps <= 0.0) {
        return video->get_output_image_param(obj, center_offset, &out, sizeof(out));
    }

    int half = window / 2;
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;

    for (int i = -half; i <= half; ++i) {
        OBJECT_IMAGE_PARAM p{};
        double off = center_offset + (double)i / fps;
        if (video->get_output_image_param(obj, off, &p, sizeof(p))) {
            sum_x += p.x;
            sum_y += p.y;
            ++count;
        }
    }

    if (count == 0) return false;
    out.x = (float)(sum_x / count);
    out.y = (float)(sum_y / count);
    return true;
}

//---------------------------------------------------------------------
//  実際の補正処理
//  ※ __try/__except と同じ関数内で C++ の try/catch は使わない方針のため、
//    ここでは例外処理をせず素直にロジックだけを書く。
//    例外の捕捉は呼び出し元の func_proc_video 側の __try/__except で行う。
//---------------------------------------------------------------------
static bool try_stabilize(FILTER_PROC_VIDEO* video) {
    if (!video || !video->param || !video->get_image_object || !video->get_output_image_param) {
        return false;
    }

    // レイヤー番号: UI表示は1始まりだが内部は0始まりなので-1する
    int layer1 = (int)std::lround(track_layer1.value) - 1;
    if (layer1 < 0) return false;

    // 追跡レイヤー1にある画像オブジェクト(現在フレーム時点のインスタンス)を取得
    // ※音声のみ/空/画像を持たないオブジェクトの場合はnullptrが返る想定
    OBJECT_HANDLE track_obj1 = video->get_image_object(layer1, 0.0);
    if (!track_obj1) {
        log_warn_once(L"[トラッキング手ブレ補正] 追跡レイヤー1に図形オブジェクトが見つかりません。補正をスキップしました。");
        return false;
    }

    // シーンのフレームレート(基準フレーム/平滑化の秒換算に使用)
    double fps = 30.0;
    if (video->scene && video->scene->scale != 0) {
        fps = (double)video->scene->rate / (double)video->scene->scale;
    }

    int window = (int)std::lround(smoothing.value);

    // 現在フレーム付近の追跡座標(平滑化込み)を取得
    OBJECT_IMAGE_PARAM cur_param1{};
    if (!get_smoothed_param(video, track_obj1, 0.0, window, fps, cur_param1)) {
        log_warn_once(L"[トラッキング手ブレ補正] 追跡レイヤー1のオブジェクトから座標を取得できません(画像オブジェクトではない可能性)。補正をスキップしました。");
        return false;
    }

    // 基準フレーム(ブレの無い基準位置)の時間(秒)を算出
    double base_time = base_frame.value / fps;
    double cur_time  = video->object ? video->object->time : 0.0;
    double offset    = base_time - cur_time; // get_output_image_paramへ渡す現在時間からのオフセット(秒)

    // 基準フレーム付近の追跡座標(平滑化込み)を取得
    // (基準フレームが追跡オブジェクトの存在範囲外の場合は失敗することがあるため
    //  その場合は補正量ゼロ扱い=現在の座標をそのまま基準として使う)
    OBJECT_IMAGE_PARAM base_param1 = cur_param1;
    get_smoothed_param(video, track_obj1, offset, window, fps, base_param1);

    // ブレ量(基準フレームからの移動量)
    double dx = cur_param1.x - base_param1.x;
    double dy = cur_param1.y - base_param1.y;

    double sign = invert.value ? -1.0 : 1.0;
    double k    = strength.value / 100.0;

    if (fix_x.value) {
        video->param->x += (float)(sign * k * dx);
    }
    if (fix_y.value) {
        video->param->y += (float)(sign * k * dy);
    }

    // ---- 回転・拡縮ブレ補正(任意): 追跡レイヤー2が使える場合のみ ----
    if (fix_rotate.value || fix_scale.value) {
        int layer2 = (int)std::lround(track_layer2.value) - 1;
        if (layer2 >= 0 && layer2 != layer1) {
            OBJECT_HANDLE track_obj2 = video->get_image_object(layer2, 0.0);
            if (track_obj2) {
                OBJECT_IMAGE_PARAM cur_param2{};
                if (get_smoothed_param(video, track_obj2, 0.0, window, fps, cur_param2)) {
                    OBJECT_IMAGE_PARAM base_param2 = cur_param2;
                    get_smoothed_param(video, track_obj2, offset, window, fps, base_param2);

                    double vx_cur  = cur_param2.x  - cur_param1.x;
                    double vy_cur  = cur_param2.y  - cur_param1.y;
                    double vx_base = base_param2.x - base_param1.x;
                    double vy_base = base_param2.y - base_param1.y;

                    double dist_cur  = std::hypot(vx_cur,  vy_cur);
                    double dist_base = std::hypot(vx_base, vy_base);

                    // 回転補正: 2点が重なっている(距離0)場合は角度が定まらないためスキップ
                    if (fix_rotate.value &&
                        (vx_cur != 0.0 || vy_cur != 0.0) && (vx_base != 0.0 || vy_base != 0.0)) {
                        double angle_cur  = std::atan2(vy_cur,  vx_cur);
                        double angle_base = std::atan2(vy_base, vx_base);
                        double d_angle = angle_cur - angle_base;

                        // -180〜180度の範囲に正規化(角度のラップアラウンド対策)
                        while (d_angle > M_PI)  d_angle -= 2.0 * M_PI;
                        while (d_angle < -M_PI) d_angle += 2.0 * M_PI;

                        double d_angle_deg = d_angle * 180.0 / M_PI;
                        double rk = rotate_strength.value / 100.0;
                        // 注: 2点の角度差は「移動量を反転して適用する」(平行移動の生データの
                        // 符号設定)の影響を受けない(両方の点が同じように反転されても角度の
                        // 差分は変わらないため)。そのため回転の符号は独立させ、
                        // 「回転方向を反転する」だけで調整する。
                        double rsign = rotate_invert.value ? -1.0 : 1.0;
                        video->param->rz += (float)(rsign * rk * d_angle_deg);
                    }

                    // 拡縮補正: 2点間の距離の変化から、基準フレームに対する拡大率を求める
                    if (fix_scale.value && dist_cur > 1e-6 && dist_base > 1e-6) {
                        // ratio<1 なら現在の方が2点間が離れている(=拡大して見えている)ので
                        // 画像を縮めて基準フレームの見た目に近づける、という向きになる。
                        double ratio = scale_invert.value ? (dist_cur / dist_base) : (dist_base / dist_cur);
                        double sk = scale_strength.value / 100.0;
                        double factor = 1.0 + (ratio - 1.0) * sk;
                        if (factor > 0.01) { // 0以下・極端な値による破綻を防止
                            video->param->sx *= (float)factor;
                            video->param->sy *= (float)factor;
                        }
                    }
                } else {
                    log_warn_once(L"[トラッキング手ブレ補正] 追跡レイヤー2のオブジェクトから座標を取得できません。回転・拡縮補正をスキップしました。");
                }
            } else {
                log_warn_once(L"[トラッキング手ブレ補正] 追跡レイヤー2に図形オブジェクトが見つかりません。回転・拡縮補正をスキップしました。");
            }
        }
    }

    // 正常に補正できたので、次に失敗した時にまた警告できるようにフラグを戻す
    g_warned_no_track_object = false;
    return true;
}

//---------------------------------------------------------------------
//  画像フィルタ処理 (エントリポイント)
//  try_stabilize()内、あるいはAviUtl2本体側で例外(C++例外・アクセス違反等)が
//  発生しても、ここで必ず捕捉し、アプリを巻き込まずに「補正なし」として
//  処理を継続する。
//---------------------------------------------------------------------
bool func_proc_video(FILTER_PROC_VIDEO* video) {
    __try {
        try_stabilize(video);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        // C++例外・アクセス違反等、いずれの異常もここで握りつぶし、
        // 補正をスキップするだけで映像処理自体は継続させる。
        // (/EHa 指定によりC++例外もここで捕捉されます)
    }
    return true;
}
