# sdf2map

Gazebo (Harmonic) の SDF world ファイルを読み込み、NDT 自己位置推定用の
3D 点群地図 (PCD) と Nav2 用の 2D 占有格子地図 (PGM + YAML) を生成する
オフライン変換ツールです。ROS 2 Jazzy 対応。

Gazebo を起動せず、libsdformat14 で world をパースして各ジオメトリの
表面を一様密度でサンプリングします。点密度が距離に依存せず、
オクルージョンによる抜けもないため、NDT のボクセル統計に適した
地図が得られます。

## 対応ジオメトリ

- box / cylinder / sphere / capsule / ellipsoid / plane
- mesh (DAE / OBJ / STL — gz-common MeshManager 経由、`<scale>` /
  `<submesh>` 対応)
- `<include>`・ネストモデル・`//pose[@relative_to]`・`<frame>` は
  libsdformat の SemanticPose で解決
- heightmap は未対応 (スキップして警告)

メッシュ URI は `model://` (GZ_SIM_RESOURCE_PATH 等 → Fuel キャッシュ
`~/.gz/fuel` の順に探索)、`file://`、絶対パス、SDF ファイルからの
相対パスを解決します。

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
| `--ascii` | off | PCD を ASCII 形式で出力 |
| `--seed <n>` | 42 | 乱数シード (出力は決定的) |
| `--occupancy-grid <file.yaml>` | なし | Nav2 地図 (YAML+PGM) も出力 |
| `--grid-resolution <m>` | 0.05 | 格子セルサイズ |
| `--slice-z-min` / `--slice-z-max <m>` | 0.1 / 1.8 | 障害物とみなす高さ帯 |
| `--grid-bounds <full\|slice>` | full | 格子範囲を全点群から取るか障害物帯のみから取るか |
| `--grid-margin <m>` | 1.0 | `--grid-bounds slice` 時の外周マージン |
| `--verbose` | off | 各モデルの解決済み world pose を表示 |

占有格子は指定高さ帯の点をセルに投影して障害物とし、地図範囲内の
残りセルは free とします (ジオメトリ既知のため unknown は生成しません)。
100×100m の ground plane を持つ world では `--grid-bounds slice` を
使うと障害物範囲+マージンのコンパクトな地図になります。

## 実運用のヒント

- **collision が床 plane 1 枚だけの world がある** (例: Fuel の Depot
  モデル)。地図が空になる場合は `--geometry visual` を使ってください。
  visual には屋根も含まれるので、必要なら `--z-max` でクロップします。
- `model://` の解決には `GZ_SIM_RESOURCE_PATH` を設定してください
  (モデルディレクトリの親を指定)。
- Fuel モデルは事前に `gz fuel download -u <URL>` でキャッシュへ
  ダウンロードしておく必要があります (本ツールは自動ダウンロード
  しません)。未解決の include は SDF エラーとして表示されます。

## テスト

```bash
colcon test --packages-select sdf2map && colcon test-result --verbose
```

サンプル world (`worlds/test_world.sdf`) には全プリミティブ・メッシュ・
ネストモデル・relative_to フレームが含まれており、動作確認に使えます。
