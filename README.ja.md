# sdf2map

[![CI](https://github.com/atinfinity/sdf2map/actions/workflows/ci.yml/badge.svg)](https://github.com/atinfinity/sdf2map/actions/workflows/ci.yml)

*[English README is here](README.md)*

Gazebo (Harmonic) の SDF world ファイルを読み込み、NDT 自己位置推定用の
3D 点群地図 (PCD) と Nav2 用の 2D 占有格子地図 (PGM + YAML) を生成する
オフライン変換ツールです。ROS 2 Jazzy 対応。

Gazebo を起動せず、libsdformat14 で world をパースして各ジオメトリの
表面を一様密度でサンプリングします。点密度が距離に依存せず、
オクルージョンによる抜けもないため、NDT のボクセル統計に適した
地図が得られます。

## 対応ジオメトリ

- box / cylinder / sphere / capsule / ellipsoid / cone / plane
- polyline (押し出し形状 — 凹多角形も ear clipping で対応)
- mesh (DAE / OBJ / STL — gz-common MeshManager 経由、`<scale>` /
  `<submesh>` 対応)
- heightmap (グレースケール画像。傾斜を考慮した面積均一サンプリング。
  DEM / GeoTIFF は未対応)
- `<include>`・ネストモデル・`//pose[@relative_to]`・`<frame>` は
  libsdformat の SemanticPose で解決

メッシュ URI は `model://` (`--model-path` → GZ_SIM_RESOURCE_PATH 等 →
Fuel キャッシュ `~/.gz/fuel` の順に探索)、`file://`、絶対パス、
SDF ファイルからの相対パスを解決します。キャッシュにない Fuel モデルは
自動ダウンロードします (`--no-download` で無効化)。

`<actor>` (アニメーション人物等) は**仕様として無視します**。動的
オブジェクトであり、静的な自己位置推定地図に含めるべきではないため
です。world に actor が含まれる場合は NOTE を表示します。

## ビルド

```bash
cd ~/dev_ws
colcon build --packages-select sdf2map
source install/setup.bash
```

## 使い方

```bash
ros2 run sdf2map sdf2map \
  --input warehouse.sdf \
  --output map.pcd \
  --voxel 0.05 \
  --occupancy-grid map.yaml
```

### 主なオプション

| オプション | 既定値 | 説明 |
|---|---|---|
| `--density <n>` | 400 | 表面サンプリング密度 [points/m²] |
| `--geometry <collision\|visual>` | collision | サンプリング対象のジオメトリ |
| `--voxel <m>` | 0 (無効) | PCD 出力の VoxelGrid リーフサイズ |
| `--z-min` / `--z-max <m>` | 無制限 | PCD 出力の高さクロップ |
| `--exclude-ground` | off | plane ジオメトリ (地面) を除外 |
| `--exclude <regex>` | なし | 名前が一致するモデルを除外 (複数指定可) |
| `--transparency-threshold <t>` | 0.95 | visual モードで透明度がこれ以上の visual を除外 |
| `--model-path <dirs>` | なし | `model://` の追加検索パス (コロン区切り・複数可) |
| `--ascii` | off | PCD を ASCII 形式で出力 |
| `--seed <n>` | 42 | 乱数シード (出力は決定的) |
| `--no-download` | off | Fuel モデルの自動ダウンロードを無効化 |
| `--publish` | off | `/sdf2map/map_cloud` に PointCloud2 を latched 配信 (RViz プレビュー) |
| `--occupancy-grid <file.yaml>` | なし | Nav2 地図 (YAML+PGM) も出力 |
| `--grid-resolution <m>` | 0.05 | 格子セルサイズ |
| `--slice-z-min` / `--slice-z-max <m>` | 0.1 / 1.8 | 障害物とみなす高さ帯 |
| `--grid-bounds <full\|slice>` | full | 格子範囲を全点群から取るか障害物帯のみから取るか |
| `--grid-margin <m>` | 1.0 | `--grid-bounds slice` 時の外周マージン |
| `--grid-close <n>` | 1 | 障害物セルのモルフォロジークロージング半径 (0で無効) |
| `--verbose` | off | 各モデルの解決済み world pose を表示 |

占有格子は指定高さ帯の点をセルに投影して障害物とし、地図範囲内の
残りセルは free とします (ジオメトリ既知のため unknown は生成しません)。
100×100m の ground plane を持つ world では `--grid-bounds slice` を
使うと障害物範囲+マージンのコンパクトな地図になります。

## 実行例

### 同梱テストworld

`worlds/test_world.sdf` は全プリミティブ・メッシュ・ネストモデル
(relative_toフレーム)・周囲壁を含みます:

```bash
ros2 run sdf2map sdf2map \
  --input worlds/test_world.sdf \
  --output test_map.pcd \
  --occupancy-grid test_map.yaml
```

| 点群地図 (PCD) | 占有格子地図 (PGM + YAML) |
|---|---|
| <img src="docs/images/test_world_cloud.png" width="420"/> | <img src="docs/images/test_world_grid.png" width="330"/> |

### 倉庫world

入力は
[kachaka_ros2_dev_kit](https://github.com/CyberAgentAILab/kachaka_ros2_dev_kit)
の `warehouse.sdf` (Fuelモデルは自動ダウンロード)。この world の
Fuel モデルは collision が貧弱なため `--geometry visual` を使い、
`--z-max` で屋根を除去、`--exclude` で人物モデルを除外しています:

```bash
ros2 run sdf2map sdf2map \
  --input warehouse.sdf \
  --output warehouse.pcd \
  --geometry visual --z-max 8 \
  --exclude 'Person|MaleVisitor|Casual' \
  --voxel 0.05 \
  --occupancy-grid warehouse.yaml
```

| 点群地図 (PCD) | 占有格子地図 (PGM + YAML) |
|---|---|
| <img src="docs/images/warehouse_cloud.png" width="420"/> | <img src="docs/images/warehouse_grid.png" width="240"/> |

この地図に対し PCL の NDT は初期誤差 0.5m / 8.6° から
9.5mm / 0.03° まで収束し、格子地図は `nav2_map_server` で
そのまま読み込めます。

## 実運用のヒント

- **collision が床 plane 1 枚だけの world がある** (例: Fuel の Depot
  モデル)。地図が空になる場合は `--geometry visual` を使ってください。
  visual には屋根も含まれるので、必要なら `--z-max` でクロップします。
- `model://` の解決には `GZ_SIM_RESOURCE_PATH` を設定するか
  `--model-path` を指定してください (モデルディレクトリの親を指定)。
- 人物などの動的オブジェクトは `--exclude 'Person|MaleVisitor'` の
  ように名前パターンで地図から除外できます。
- Fuel モデルはキャッシュになければ自動ダウンロードされます。
  オフライン環境では事前に `gz fuel download -u <URL>` で取得した上で
  `--no-download` を指定してください。

## テスト

```bash
colcon test --packages-select sdf2map && colcon test-result --verbose
```

サンプル world (`worlds/test_world.sdf`) には全プリミティブ・メッシュ・
ネストモデル・relative_to フレームが含まれており、動作確認に使えます。

## ロードマップ

計画済み機能は実装完了しています。判断待ち事項と任意の将来項目は
[docs/ROADMAP.md](docs/ROADMAP.md) を参照してください。
