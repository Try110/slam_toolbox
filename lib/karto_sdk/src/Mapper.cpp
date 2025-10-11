/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <karto_sdk/Types.h>
#include <math.h>
#include <assert.h>
#include <boost/serialization/vector.hpp>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <set>
#include <list>
#include <iterator>
#include <map>
#include <vector>
#include <utility>
#include <algorithm>
#include <string>

#include "karto_sdk/Mapper.h"

BOOST_CLASS_EXPORT(karto::MapperGraph);
BOOST_CLASS_EXPORT(karto::Graph<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::EdgeLabel);
BOOST_CLASS_EXPORT(karto::LinkInfo);
BOOST_CLASS_EXPORT(karto::Edge<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::Vertex<karto::LocalizedRangeScan>);
BOOST_CLASS_EXPORT(karto::MapperSensorManager)
BOOST_CLASS_EXPORT(karto::Mapper)
namespace karto
{

// enable this for verbose debug information
// #define KARTO_DEBUG

  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Manages the scan data for a device
 */
class ScanManager
{
public:
  /**
   * Default constructor
   */
  ScanManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
  : m_pLastScan(NULL),
    m_RunningBufferMaximumSize(runningBufferMaximumSize),
    m_RunningBufferMaximumDistance(runningBufferMaximumDistance),
    m_NextStateId(0)
  {
  }

  ScanManager() {}

  /**
   * Destructor
   */
  virtual ~ScanManager()
  {
    Clear();
  }

public:
  /**
   * Adds scan to vector of processed scans tagging scan with given unique id
   * @param pScan
   */
  inline void AddScan(LocalizedRangeScan * pScan, kt_int32s uniqueId)
  {
    // assign state id to scan
    pScan->SetStateId(m_NextStateId);

    // assign unique id to scan
    pScan->SetUniqueId(uniqueId);

    // add it to scan buffer
    m_Scans.insert({pScan->GetStateId(), pScan});
    m_NextStateId++;
  }

  /**
   * Gets last scan
   * @param deviceId
   * @return last localized range scan
   */
  inline LocalizedRangeScan * GetLastScan()
  {
    return m_pLastScan;
  }

  /**
   * Clears last scan
   * @param deviceId
   */
  inline void ClearLastScan()
  {
    m_pLastScan = NULL;
  }

  /**
   * Sets the last scan
   * @param pScan
   */
  void SetLastScan(LocalizedRangeScan * pScan)
  {
    m_pLastScan = pScan;
  }

  /**
   * Gets scans
   * @return scans
   */
  inline LocalizedRangeScanMap & GetScans()
  {
    return m_Scans;
  }

  /**
   * Gets running scans
   * @return running scans
   */
  inline LocalizedRangeScanVector & GetRunningScans()
  {
    return m_RunningScans;
  }

  /**
   * Gets running scan buffer size
   * @return running scan buffer size
   */
  inline kt_int32u & GetRunningScanBufferSize()
  {
    return m_RunningBufferMaximumSize;
  }

  /**
   * Sets running scan buffer size
   * @param rScanBufferSize
   */
  void SetRunningScanBufferSize(const kt_int32u & rScanBufferSize)
  {
    m_RunningBufferMaximumSize = rScanBufferSize;
  }

  /**
   * Sets running scan buffer maximum distance
   * @param rScanBufferMaxDistance
   */
  void SetRunningScanBufferMaximumDistance(const kt_int32u & rScanBufferMaxDistance)
  {
    m_RunningBufferMaximumDistance = rScanBufferMaxDistance;
  }

  /**
   * Adds scan to vector of running scans
   * @param pScan
   */
  void AddRunningScan(LocalizedRangeScan * pScan)
  {
    m_RunningScans.push_back(pScan);

    // vector has at least one element (first line of this function), so this is valid
    Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
    Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

    // cap vector size and remove all scans from front of vector that are too far from end of vector
    kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(
      backScanPose.GetPosition());
    while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
      squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
    {
      // remove front of running scans
      m_RunningScans.erase(m_RunningScans.begin());

      // recompute stats of running scans
      frontScanPose = m_RunningScans.front()->GetSensorPose();
      backScanPose = m_RunningScans.back()->GetSensorPose();
      squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
    }
  }

  /**
   * Finds and replaces a scan from m_scans with NULL
   * @param pScan
   */
  void RemoveScan(LocalizedRangeScan * pScan)
  {
    LocalizedRangeScanMap::iterator it = m_Scans.find(pScan->GetStateId());
    if (it != m_Scans.end()) {
      it->second = NULL;
      m_Scans.erase(it);
    } else {
      std::cout << "Remove Scan: Failed to find scan in m_Scans" << std::endl;
    }
  }

  /**
   * Clears the vector of running scans
   */
  void ClearRunningScans()
  {
    m_RunningScans.clear();
  }

  /**
   * Deletes data of this buffered device
   */
  void Clear()
  {
    m_Scans.clear();
    m_RunningScans.clear();
  }

private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_NVP(m_Scans);
    ar & BOOST_SERIALIZATION_NVP(m_RunningScans);
    ar & BOOST_SERIALIZATION_NVP(m_pLastScan);
    ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumSize);
    ar & BOOST_SERIALIZATION_NVP(m_RunningBufferMaximumDistance);
    ar & BOOST_SERIALIZATION_NVP(m_NextStateId);
  }

private:
  LocalizedRangeScanMap m_Scans;
  LocalizedRangeScanVector m_RunningScans;
  LocalizedRangeScan * m_pLastScan;
  kt_int32u m_NextStateId;

  kt_int32u m_RunningBufferMaximumSize;
  kt_double m_RunningBufferMaximumDistance;
};    // ScanManager

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void MapperSensorManager::RegisterSensor(const Name & rSensorName)
{
  if (GetScanManager(rSensorName) == NULL) {
    m_ScanManagers[rSensorName] = new ScanManager(
      m_RunningBufferMaximumSize,
      m_RunningBufferMaximumDistance);
  }
}


/**
 * Gets scan from given device with given ID
 * @param rSensorName
 * @param scanNum
 * @return localized range scan
 */
LocalizedRangeScan * MapperSensorManager::GetScan(const Name & rSensorName, kt_int32s scanIndex)
{
  ScanManager * pScanManager = GetScanManager(rSensorName);
  if (pScanManager != NULL) {
    LocalizedRangeScanMap::iterator it = pScanManager->GetScans().find(scanIndex);
    if (it != pScanManager->GetScans().end()) {
      return it->second;
    } else {
      return nullptr;
    }
  }

  assert(false);
  return NULL;
}

/**
 * Gets last scan of given device
 * @param pLaserRangeFinder
 * @return last localized range scan of device
 */
inline LocalizedRangeScan * MapperSensorManager::GetLastScan(const Name & rSensorName)
{
  RegisterSensor(rSensorName);

  return GetScanManager(rSensorName)->GetLastScan();
}

/**
 * Sets the last scan of device of given scan
 * @param pScan
 */
void MapperSensorManager::SetLastScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->SetLastScan(pScan);
}

/**
 * Clears the last scan of device of given scan
 * @param pScan
 */
void MapperSensorManager::ClearLastScan(LocalizedRangeScan* pScan)
{
  GetScanManager(pScan)->ClearLastScan();
}

/**
 * Clears the last scan of device name
 * @param pScan
 */
void MapperSensorManager::ClearLastScan(const Name& name)
{
  GetScanManager(name)->ClearLastScan();
}

/**
 * Adds scan to scan vector of device that recorded scan
 * @param pScan
 */
void MapperSensorManager::AddScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->AddScan(pScan, m_NextScanId);
  m_Scans.insert({m_NextScanId, pScan});
  m_NextScanId++;
}

/**
 * Adds scan to running scans of device that recorded scan
 * @param pScan
 */
inline void MapperSensorManager::AddRunningScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->AddRunningScan(pScan);
}

/**
 * Finds and replaces a scan from m_Scans with NULL
 * @param pScan
 */
void MapperSensorManager::RemoveScan(LocalizedRangeScan * pScan)
{
  GetScanManager(pScan)->RemoveScan(pScan);

  LocalizedRangeScanMap::iterator it = m_Scans.find(pScan->GetUniqueId());
  if (it != m_Scans.end()) {
    it->second = NULL;
    m_Scans.erase(it);
  } else {
    std::cout << "RemoveScan: Failed to find scan in m_Scans" << std::endl;
  }
}

/**
 * Gets scans of device
 * @param rSensorName
 * @return scans of device
 */
inline LocalizedRangeScanMap & MapperSensorManager::GetScans(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetScans();
}

/**
 * Gets running scans of device
 * @param rSensorName
 * @return running scans of device
 */
inline LocalizedRangeScanVector & MapperSensorManager::GetRunningScans(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetRunningScans();
}

void MapperSensorManager::ClearRunningScans(const Name & rSensorName)
{
  GetScanManager(rSensorName)->ClearRunningScans();
}

inline kt_int32u MapperSensorManager::GetRunningScanBufferSize(const Name & rSensorName)
{
  return GetScanManager(rSensorName)->GetRunningScanBufferSize();
}

void MapperSensorManager::SetRunningScanBufferSize(kt_int32u rScanBufferSize)
{
  m_RunningBufferMaximumSize = rScanBufferSize;

  std::vector<Name> names = GetSensorNames();
  for (uint i = 0; i != names.size(); i++) {
    GetScanManager(names[i])->SetRunningScanBufferSize(rScanBufferSize);
  }
}

void MapperSensorManager::SetRunningScanBufferMaximumDistance(kt_double rScanBufferMaxDistance)
{
  m_RunningBufferMaximumDistance = rScanBufferMaxDistance;

  std::vector<Name> names = GetSensorNames();
  for (uint i = 0; i != names.size(); i++) {
    GetScanManager(names[i])->SetRunningScanBufferMaximumDistance(rScanBufferMaxDistance);
  }
}

/**
 * Gets all scans of all devices
 * @return all scans of all devices
 */
LocalizedRangeScanVector MapperSensorManager::GetAllScans()
{
  LocalizedRangeScanVector scans;

  forEach(ScanManagerMap, &m_ScanManagers)
  {
    LocalizedRangeScanMap & rScans = iter->second->GetScans();

    LocalizedRangeScanMap::iterator it;
    for (it = rScans.begin(); it != rScans.end(); ++it) {
      scans.push_back(it->second);
    }
  }

  return scans;
}

/**
 * Deletes all scan managers of all devices
 */
void MapperSensorManager::Clear()
{
//    SensorManager::Clear();

  forEach(ScanManagerMap, &m_ScanManagers)
  {
    delete iter->second;
    iter->second = nullptr;
  }

  m_ScanManagers.clear();
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

ScanMatcher::~ScanMatcher()
{
  if (m_pCorrelationGrid) {
    delete m_pCorrelationGrid;
  }
  if (m_pSearchSpaceProbs) {
    delete m_pSearchSpaceProbs;
  }
  if (m_pGridLookup) {
    delete m_pGridLookup;
  }
}

ScanMatcher * ScanMatcher::Create(
  Mapper * pMapper, kt_double searchSize, kt_double resolution,
  kt_double smearDeviation, kt_double rangeThreshold)
{
  // invalid parameters
  if (resolution <= 0) {
    return NULL;
  }
  if (searchSize <= 0) {
    return NULL;
  }
  if (smearDeviation < 0) {
    return NULL;
  }
  if (rangeThreshold <= 0) {
    return NULL;
  }

  assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));

  // calculate search space in grid coordinates
  kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);

  // compute requisite size of correlation grid (pad grid so that scan
  // points can't fall off the grid
  // if a scan is on the border of the search space)
  kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

  kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

  // create correlation grid
  assert(gridSize % 2 == 1);
  CorrelationGrid * pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution,
      smearDeviation);

  // create search space probabilities
  Grid<kt_double> * pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize,
      searchSpaceSideSize, resolution);

  ScanMatcher * pScanMatcher = new ScanMatcher(pMapper);
  pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
  pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;
  pScanMatcher->m_pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);

  return pScanMatcher;
}

/**
 * 将给定的扫描与一组基准扫描进行匹配
 * @param pScan 需要进行扫描匹配的目标扫描
 * @param rBaseScans 基准扫描集合，用于在相关性网格中标记占用单元格
 * @param rMean 输出参数，返回匹配的最佳位姿（均值）
 * @param rCovariance 输出参数，返回匹配结果的协方差矩阵
 * @param doPenalize 是否对远离搜索中心的匹配结果进行惩罚
 * @param doRefineMatch 是否在粗匹配效果良好时进行精细匹配（默认为true）
 * @return 匹配响应强度值
 */
template<class T>
kt_double ScanMatcher::MatchScan(
  LocalizedRangeScan * pScan, const T & rBaseScans, Pose2 & rMean,
  Matrix3 & rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
{
  ///////////////////////////////////////
  // 设置扫描位姿为网格中心

  // 1. 获取扫描位置
  Pose2 scanPose = pScan->GetSensorPose();

  // 如果扫描没有读数，则无法进行扫描匹配
  // 在这种情况下，基于调整后的里程计读数作为最佳猜测位姿
  if (pScan->GetNumberOfRangeReadings() == 0) {
    rMean = scanPose;

    // 设置最大协方差值
    rCovariance(0, 0) = MAX_VARIANCE;    // XX方向方差
    rCovariance(1, 1) = MAX_VARIANCE;    // YY方向方差
    rCovariance(2, 2) =
      4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue());    // 角度方向方差

    return 0.0;  // 返回0响应值
  }

  // 2. 获取相关性网格的尺寸
  Rectangle2<kt_int32s> roi = m_pCorrelationGrid->GetROI();

  // 3. 计算偏移量（以米为单位 - 左下角坐标）
  Vector2<kt_double> offset;
  offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->GetResolution()));
  offset.SetY(scanPose.GetY() -
    (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->GetResolution()));

  // 4. 设置偏移量
  m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

  ///////////////////////////////////////

  // 建立相关性网格，将基准扫描点标记为占用
  AddScans(rBaseScans, scanPose.GetPosition());

  // 计算在每个方向上搜索的距离
  Vector2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(),
    m_pSearchSpaceProbs->GetHeight());
  Vector2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) *
    m_pCorrelationGrid->GetResolution(),
    0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->GetResolution());

  // 粗搜索只检查每个维度的一半单元格
  Vector2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->GetResolution(),
    2 * m_pCorrelationGrid->GetResolution());

  // 实际的扫描匹配过程 - 粗匹配
  kt_double bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset,
      coarseSearchResolution,
      m_pMapper->m_pCoarseSearchAngleOffset->GetValue(),
      m_pMapper->m_pCoarseAngleResolution->GetValue(),
      doPenalize, rMean, rCovariance, false);

  // 如果启用了响应扩展功能且粗匹配未找到有效结果
  if (m_pMapper->m_pUseResponseExpansion->GetValue() == true) {
    if (math::DoubleEqual(bestResponse, 0.0)) {
#ifdef KARTO_DEBUG
      std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
      // 尝试增加搜索角度偏移量（每次增加20度）并重新匹配
      kt_double newSearchAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
      for (kt_int32u i = 0; i < 3; i++) {
        newSearchAngleOffset += math::DegreesToRadians(20);

        bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset, coarseSearchResolution,
            newSearchAngleOffset, m_pMapper->m_pCoarseAngleResolution->GetValue(),
            doPenalize, rMean, rCovariance, false);

        // 如果找到非零响应则停止扩展搜索
        if (math::DoubleEqual(bestResponse, 0.0) == false) {
          break;
        }
      }

#ifdef KARTO_DEBUG
      if (math::DoubleEqual(bestResponse, 0.0)) {
        std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
      }
#endif
    }
  }

  // 如果需要进行精细匹配
  if (doRefineMatch) {
    // 精细搜索的偏移量是粗搜索分辨率的一半
    Vector2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
    Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(),
      m_pCorrelationGrid->GetResolution());

    // 执行精细匹配，以粗匹配结果为中心进行更精细的搜索
    bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
        0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(),
        m_pMapper->m_pFineSearchAngleOffset->GetValue(),
        doPenalize, rMean, rCovariance, true);
  }

#ifdef KARTO_DEBUG
  std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse <<
    ",  VARIANCE = " <<
    rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif
  // 确保返回的角度在[-π, π]范围内
  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));
  std::cout << "[MatchScan] Match response: " << bestResponse << std::endl;
  return bestResponse;  // 返回最佳匹配响应值
}


/**
 * TBB 并行入口：每个线程被分配一个 y 值，
 * 负责把“当前 y 行”里所有 (x,θ) 候选位姿的匹配得分算出来并写回全局数组。
 */
void ScanMatcher::operator()(const kt_double & y) const
{
  kt_int32u poseResponseCounter;   // 在 m_pPoseResponse[] 中的线性下标
  kt_int32u x_pose;                // 当前 x 序号
  /* 通过 find 拿到 y 在 m_yPoses 中的序号，用于后面计算线性下标 */
  kt_int32u y_pose = std::find(m_yPoses.begin(), m_yPoses.end(), y) - m_yPoses.begin();

  const kt_int32u size_x = m_xPoses.size();   // x 方向候选个数，用于偏移量计算

  /* 计算实际世界坐标 & 离中心距离平方，后面做距离惩罚用 */
  kt_double newPositionY = m_rSearchCenter.GetY() + y;
  kt_double squareY      = math::Square(y);   // y²

  /* --------------- 遍历同一行里的所有 x --------------- */
  for (std::vector<kt_double>::const_iterator xIter = m_xPoses.begin();
       xIter != m_xPoses.end(); ++xIter)
  {
    x_pose = std::distance(m_xPoses.begin(), xIter);  // x 序号
    kt_double x = *xIter;
    kt_double newPositionX = m_rSearchCenter.GetX() + x;
    kt_double squareX      = math::Square(x);         // x²

    /* 把 (x,y) 转成栅格坐标，再拿一维栅格索引，后面直接查表 */
    Vector2<kt_int32s> gridPoint =
      m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(newPositionX, newPositionY));
    kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
    assert(gridIndex >= 0);

    /* --------------- 遍历所有角度 --------------- */
    kt_double angle = 0.0;
    kt_double startAngle = m_rSearchCenter.GetHeading() - m_searchAngleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < m_nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * m_searchAngleResolution;

      /* 1. 查表拿到当前 (angle,grid) 的原始相关得分 [0,occMax] */
      kt_double response = GetResponse(angleIndex, gridIndex);

      /* 2. 若开启惩罚，用简单高斯模型把“离中心越远”的候选降权 */
      if (m_doPenalize && (math::DoubleEqual(response, 0.0) == false))
      {
        kt_double squaredDistance = squareX + squareY;   // 离中心距离²
        kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN *
          squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());
        distancePenalty = math::Maximum(distancePenalty,
                           m_pMapper->m_pMinimumDistancePenalty->GetValue());
        kt_double deltaAngle = math::NormalizeAngle(angle - m_rSearchCenter.GetHeading());
        kt_double squaredAngleDistance = math::Square(deltaAngle);
        // kt_double squaredAngleDistance = math::Square(angle - m_rSearchCenter.GetHeading());
        kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN *
          squaredAngleDistance / m_pMapper->m_pAngleVariancePenalty->GetValue());
        anglePenalty = math::Maximum(anglePenalty,
                         m_pMapper->m_pMinimumAnglePenalty->GetValue());

        response *= (distancePenalty * anglePenalty);   // 最终得分
      }

      /* 3. 计算线性下标，把 (response, 位姿) 写回全局数组供主线程归并 */
      poseResponseCounter = (y_pose * size_x + x_pose) * (m_nAngles) + angleIndex;
      m_pPoseResponse[poseResponseCounter] =
        std::pair<kt_double, Pose2>(response,
                                     Pose2(newPositionX, newPositionY,
                                           math::NormalizeAngle(angle)));
    } // for angle
  } // for x
}

/**
 * Finds the best pose for the scan centering the search in the correlation grid
 * at the given pose and search in the space by the vector and angular offsets
 * in increments of the given resolutions
 * @param rScan scan to match against correlation grid
 * @param rSearchCenter the center of the search space
 * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
 * @param rSearchSpaceResolution how fine a granularity to search in the search space
 * @param searchAngleOffset searches poses in the angles offset by this angle around search center
 * @param searchAngleResolution how fine a granularity to search in the angular search space
 * @param doPenalize whether to penalize matches further from the search center
 * @param rMean output parameter of mean (best pose) of match
 * @param rCovariance output parameter of covariance of match
 * @param doingFineMatch whether to do a finer search after coarse search
 * @return strength of response
 */
/**
 * 在相关栅格里以 rSearchCenter 为中心，按给定步长遍历 2D+角度 搜索空间，
 * 找出使匹配得分最高的位姿，并计算其协方差。
 */
kt_double ScanMatcher::CorrelateScan(
  LocalizedRangeScan * pScan,               // 当前激光帧
  const Pose2 & rSearchCenter,              // 搜索中心位姿（粗匹配时通常是里程计/上一帧位姿）
  const Vector2<kt_double> & rSearchSpaceOffset,    // x/y 方向搜索半宽
  const Vector2<kt_double> & rSearchSpaceResolution,// x/y 步长
  kt_double searchAngleOffset,              // 角度搜索半宽
  kt_double searchAngleResolution,          // 角度步长
  kt_bool doPenalize,                       // 是否对离中心远的候选位姿降分
  Pose2 & rMean,                            // 输出：最优位姿
  Matrix3 & rCovariance,                    // 输出：对应协方差
  kt_bool doingFineMatch)                   // true=精搜索，false=粗搜索
{
  assert(searchAngleResolution != 0.0);

  /* 1. 预计算每个角度下激光点对应的栅格索引，后面直接查表 */
  m_pGridLookup->ComputeOffsets(pScan,
    rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);

  /* 2. 若是粗搜索，要初始化概率栅格，用来后续算位置协方差 */
  if (!doingFineMatch) {
    m_pSearchSpaceProbs->Clear();
    Vector2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
    m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
  }

  /* 3. 生成 x 方向搜索序列 [-offset, +offset]，步长 resolution */
  m_xPoses.clear();
  kt_int32u nX = static_cast<kt_int32u>(
        math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
  kt_double startX = -rSearchSpaceOffset.GetX();
  for (kt_int32u xIndex = 0; xIndex < nX; xIndex++) {
    m_xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
  }
  assert(math::DoubleEqual(m_xPoses.back(), -startX));

  /* 4. 同理生成 y 方向搜索序列 */
  m_yPoses.clear();
  kt_int32u nY = static_cast<kt_int32u>(
        math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
  kt_double startY = -rSearchSpaceOffset.GetY();
  for (kt_int32u yIndex = 0; yIndex < nY; yIndex++) {
    m_yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
  }
  assert(math::DoubleEqual(m_yPoses.back(), -startY));

  /* 5. 计算角度个数，申请一维数组存所有候选位姿的得分 */
  kt_int32u nAngles =
    static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);
  kt_int32u poseResponseSize = static_cast<kt_int32u>(m_xPoses.size() * m_yPoses.size() * nAngles);
  m_pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];

  /* 6. 记录搜索参数，供并行算子使用 */
  Vector2<kt_int32s> startGridPoint =
    m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + startX,
                                                         rSearchCenter.GetY() + startY));
  m_rSearchCenter        = rSearchCenter;
  m_searchAngleOffset    = searchAngleOffset;
  m_nAngles              = nAngles;
  m_searchAngleResolution= searchAngleResolution;
  m_doPenalize           = doPenalize;

  /* 7. TBB 并行：对每个 y 行调用 operator()(y)，内部走完 x 与 angle 三层循环，在这里边计算 m_pPoseResponse */
  tbb::parallel_for_each(m_yPoses, (*this));

  /* 8. 并行结束后，在一维数组里找最大得分 bestResponse */
  kt_double bestResponse = -1;
  for (kt_int32u i = 0; i < poseResponseSize; i++) {
    bestResponse = math::Maximum(bestResponse, m_pPoseResponse[i].first);

    /* 粗搜索时，把每个栅格的最高得分记下来，后面算位置协方差要用 */
    if (!doingFineMatch) {
      const Pose2 & rPose = m_pPoseResponse[i].second;
      Vector2<kt_int32s> grid = m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());
      kt_double * ptr = nullptr;
      try {
        ptr = (kt_double *)(m_pSearchSpaceProbs->GetDataPointer(grid));
      } catch (...) {
        throw std::runtime_error("Mapper FATAL ERROR - "
                "unable to get pointer in probability search!");
      }
      if (ptr == NULL) {
        throw std::runtime_error("Mapper FATAL ERROR - "
                "Index out of range in probability search!");
      }
      *ptr = math::Maximum(m_pPoseResponse[i].first, *ptr);
    }
  }

  /* 9. 可能多个候选位姿得分都等于 bestResponse，求平均位姿 */
  Vector2<kt_double> averagePosition(0, 0);
  kt_double thetaX = 0.0, thetaY = 0.0;
  kt_int32s averagePoseCount = 0;
  for (kt_int32u i = 0; i < poseResponseSize; i++) {
    if (math::DoubleEqual(m_pPoseResponse[i].first, bestResponse)) {
      averagePosition += m_pPoseResponse[i].second.GetPosition();
      kt_double heading = m_pPoseResponse[i].second.GetHeading();
      thetaX += cos(heading);
      thetaY += sin(heading);
      averagePoseCount++;
    }
  }

  Pose2 averagePose;
  if (averagePoseCount > 0) {
    averagePosition /= averagePoseCount;

    thetaX /= averagePoseCount;
    thetaY /= averagePoseCount;

    averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
  } else {
    throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
  }

  /* 10. 释放临时数组 */
  delete[] m_pPoseResponse;
  m_pPoseResponse = nullptr;

  /* 11. 根据粗/精标志，算位置或角度协方差 */
  if (!doingFineMatch) {
    /* 粗搜索：用概率栅格里所有高分格算 x-y 协方差 */
    ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter,
                                rSearchSpaceOffset, rSearchSpaceResolution,
                                searchAngleResolution, rCovariance);
  } else {
    /* 精搜索：只算角度协方差，位置协方差沿用粗搜索结果 */
    ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter,
                             searchAngleOffset, searchAngleResolution,
                             rCovariance);
  }

  /* 12. 返回最优位姿与得分 */
  rMean = averagePose;

  if (bestResponse > 1.0) bestResponse = 1.0;
  assert(math::InRange(bestResponse, 0.0, 1.0));
  assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  std::cout << "[CorrelateScan] Match response: " << bestResponse << std::endl;
  return bestResponse;
}

/**
 * Computes the positional covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchSpaceOffset
 * @param rSearchSpaceResolution
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputePositionalCovariance(
  const Pose2 & rBestPose, kt_double bestResponse,
  const Pose2 & rSearchCenter,
  const Vector2<kt_double> & rSearchSpaceOffset,
  const Vector2<kt_double> & rSearchSpaceResolution,
  kt_double searchAngleResolution, Matrix3 & rCovariance)
{
  // reset covariance to identity matrix
  rCovariance.SetToIdentity();

  // if best response is vary small return max variance
  if (bestResponse < KT_TOLERANCE) {
    rCovariance(0, 0) = MAX_VARIANCE;    // XX
    rCovariance(1, 1) = MAX_VARIANCE;    // YY
    rCovariance(2, 2) = 4 * math::Square(searchAngleResolution);    // TH*TH

    return;
  }

  kt_double accumulatedVarianceXX = 0;
  kt_double accumulatedVarianceXY = 0;
  kt_double accumulatedVarianceYY = 0;
  kt_double norm = 0;

  kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
  kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();

  kt_double offsetX = rSearchSpaceOffset.GetX();
  kt_double offsetY = rSearchSpaceOffset.GetY();

  kt_int32u nX =
    static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
  kt_double startX = -offsetX;
  assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));

  kt_int32u nY =
    static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
  kt_double startY = -offsetY;
  assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));

  for (kt_int32u yIndex = 0; yIndex < nY; yIndex++) {
    kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();

    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++) {
      kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();

      Vector2<kt_int32s> gridPoint =
        m_pSearchSpaceProbs->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + x,
          rSearchCenter.GetY() + y));
      kt_double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));

      // response is not a low response
      if (response >= (bestResponse - 0.1)) {
        norm += response;
        accumulatedVarianceXX += (math::Square(x - dx) * response);
        accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
        accumulatedVarianceYY += (math::Square(y - dy) * response);
      }
    }
  }

  if (norm > KT_TOLERANCE) {
    kt_double varianceXX = accumulatedVarianceXX / norm;
    kt_double varianceXY = accumulatedVarianceXY / norm;
    kt_double varianceYY = accumulatedVarianceYY / norm;
    kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);

    // lower-bound variances so that they are not too small;
    // ensures that links are not too tight
    kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
    kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
    varianceXX = math::Maximum(varianceXX, minVarianceXX);
    varianceYY = math::Maximum(varianceYY, minVarianceYY);

    // increase variance for poorer responses
    kt_double multiplier = 1.0 / bestResponse;
    rCovariance(0, 0) = varianceXX * multiplier;
    rCovariance(0, 1) = varianceXY * multiplier;
    rCovariance(1, 0) = varianceXY * multiplier;
    rCovariance(1, 1) = varianceYY * multiplier;
    rCovariance(2, 2) = varianceTHTH;    // this value will be set in ComputeAngularCovariance
  }

  // if values are 0, set to MAX_VARIANCE
  // values might be 0 if points are too sparse and thus don't hit other points
  if (math::DoubleEqual(rCovariance(0, 0), 0.0)) {
    rCovariance(0, 0) = MAX_VARIANCE;
  }

  if (math::DoubleEqual(rCovariance(1, 1), 0.0)) {
    rCovariance(1, 1) = MAX_VARIANCE;
  }
}

/**
 * Computes the angular covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchAngleOffset
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputeAngularCovariance(
  const Pose2 & rBestPose,
  kt_double bestResponse,
  const Pose2 & rSearchCenter,
  kt_double searchAngleOffset,
  kt_double searchAngleResolution,
  Matrix3 & rCovariance)
{
  // NOTE: do not reset covariance matrix

  // normalize angle difference
  kt_double bestAngle = math::NormalizeAngleDifference(
    rBestPose.GetHeading(), rSearchCenter.GetHeading());

  Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
  kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

  kt_int32u nAngles =
    static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

  kt_double angle = 0.0;
  kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

  kt_double norm = 0.0;
  kt_double accumulatedVarianceThTh = 0.0;
  for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++) {
    angle = startAngle + angleIndex * searchAngleResolution;
    kt_double response = GetResponse(angleIndex, gridIndex);

    // response is not a low response
    if (response >= (bestResponse - 0.1)) {
      norm += response;
      accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
    }
  }
  assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

  if (norm > KT_TOLERANCE) {
    if (accumulatedVarianceThTh < KT_TOLERANCE) {
      accumulatedVarianceThTh = math::Square(searchAngleResolution);
    }

    accumulatedVarianceThTh /= norm;
  } else {
    accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
  }

  rCovariance(2, 2) = accumulatedVarianceThTh;
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanVector & rScans, Vector2<kt_double> viewPoint)
{
  m_pCorrelationGrid->Clear();

  // add all scans to grid
  const_forEach(LocalizedRangeScanVector, &rScans)
  {
    if (*iter == NULL) {
      continue;
    }

    AddScan(*iter, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanMap & rScans, Vector2<kt_double> viewPoint)
{
  m_pCorrelationGrid->Clear();

  // add all scans to grid
  const_forEach(LocalizedRangeScanMap, &rScans)
  {
    if (iter->second == NULL) {
      continue;
    }

    AddScan(iter->second, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::AddScan(
  LocalizedRangeScan * pScan, const Vector2<kt_double> & rViewPoint,
  kt_bool doSmear)
{
  PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

  // put in all valid points
  const_forEach(PointVectorDouble, &validPoints)
  {
    Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(*iter);
    if (!math::IsUpTo(gridPoint.GetX(), m_pCorrelationGrid->GetROI().GetWidth()) ||
      !math::IsUpTo(gridPoint.GetY(), m_pCorrelationGrid->GetROI().GetHeight()))
    {
      // point not in grid
      continue;
    }

    int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    // set grid cell as occupied
    if (m_pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied) {
      // value already set
      continue;
    }

    m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

    // smear grid
    if (doSmear == true) {
      m_pCorrelationGrid->SmearPoint(gridPoint);
    }
  }
}

/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param pScan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble ScanMatcher::FindValidPoints(
  LocalizedRangeScan * pScan,
  const Vector2<kt_double> & rViewPoint) const
{
  const PointVectorDouble & rPointReadings = pScan->GetPointReadings();

  // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
  const kt_double minSquareDistance = math::Square(0.1);    // in m^2

  // this iterator lags from the main iterator adding points only when the points are on
  // the same side as the viewpoint
  PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
  PointVectorDouble validPoints;

  Vector2<kt_double> firstPoint;
  kt_bool firstTime = true;
  const_forEach(PointVectorDouble, &rPointReadings)
  {
    Vector2<kt_double> currentPoint = *iter;

    if (firstTime && !std::isnan(currentPoint.GetX()) && !std::isnan(currentPoint.GetY())) {
      firstPoint = currentPoint;
      firstTime = false;
    }

    Vector2<kt_double> delta = firstPoint - currentPoint;
    if (delta.SquaredLength() > minSquareDistance) {
      // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
      // Which computes the direction of rotation, if the rotation is counterclock
      // wise then we are looking at data we should keep. If it's negative rotation
      // we should not included in in the matching
      // have enough distance, check viewpoint
      double a = rViewPoint.GetY() - firstPoint.GetY();
      double b = firstPoint.GetX() - rViewPoint.GetX();
      double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
      double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

      // reset beginning point
      firstPoint = currentPoint;

      if (ss < 0.0) {  // wrong side, skip and keep going
        trailingPointIter = iter;
      } else {
        for (; trailingPointIter != iter; ++trailingPointIter) {
          validPoints.push_back(*trailingPointIter);
        }
      }
    }
  }

  return validPoints;
}

  /**
   * 给定“角度索引”与“栅格起点”，累加所有激光点在该角度下命中栅格的占用值，
   * 并归一化到 [0,1] 作为匹配得分。
   * @param angleIndex        预计算角度表索引
   * @param gridPositionIndex 候选位姿对应的栅格起点一维索引
   * @return response         匹配得分 [0,1]，越大表示越吻合
   */
  kt_double ScanMatcher::GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const
{
  kt_double response = 0.0;

  /* 直接拿栅格内存首指针，后面用偏移量累加，避免重复计算二维索引 */
  kt_int8u * pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

  /* 取出该角度下“每个激光点→栅格偏移量”的预计算表 */
  const LookupArray * pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
  assert(pOffsets != NULL);

  kt_int32u nPoints = pOffsets->GetSize();   // 有效激光点个数
  if (nPoints == 0) {
    return response;                         // 无点直接返回 0
  }

  /* 累加占用值：遍历所有激光点 */
  kt_int32s * pAngleIndexPointer = pOffsets->GetArrayPointer();
  for (kt_int32u i = 0; i < nPoints; i++) {
    /* 当前点相对于栅格起点的偏移量（已预计算，避免实时三角函数） */
    kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];

    /* 越界或无效点跳过，防止访问栅格外内存 */
    if (!math::IsUpTo(pointGridIndex,
      m_pCorrelationGrid->GetDataSize()) || pAngleIndexPointer[i] == INVALID_SCAN)
    {
      continue;
    }

    /* 累加该点所在栅格的占用值（0=自由，255=占用） */
    response += pByte[pAngleIndexPointer[i]];
  }

  /* 归一化：除以“点数 * 最大可能占用值”，把结果压到 [0,1] */
  response /= (nPoints * GridStates_Occupied);
  assert(fabs(response) <= 1.0);

  return response;
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
class BreadthFirstTraversal : public GraphTraversal<T>
{
public:
  /**
   * Constructs a breadth-first traverser for the given graph
   */
  BreadthFirstTraversal()
  {
  }
  explicit BreadthFirstTraversal(Graph<T> * pGraph)
  : GraphTraversal<T>(pGraph)
  {
  }

  /**
   * Destructor
   */
  virtual ~BreadthFirstTraversal()
  {
  }

public:
  /**
   * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
   * @param pStartVertex
   * @param pVisitor
   * @return visited vertice scans
   */
  virtual std::vector<T *> TraverseForScans(Vertex<T> * pStartVertex, Visitor<T> * pVisitor)
  {
    std::vector<Vertex<T> *> validVertices = TraverseForVertices(pStartVertex, pVisitor);

    std::vector<T *> objects;
    forEach(typename std::vector<Vertex<T> *>, &validVertices)
    {
      objects.push_back((*iter)->GetObject());
    }

    return objects;
  }

  /**
   * Traverse the graph starting with the given vertex; applies the visitor to visited nodes
   * @param pStartVertex
   * @param pVisitor
   * @return visited vertices
   */
  virtual std::vector<Vertex<T> *> TraverseForVertices(
    Vertex<T> * pStartVertex,
    Visitor<T> * pVisitor)
  {
    std::queue<Vertex<T> *> toVisit;
    std::set<Vertex<T> *> seenVertices;
    std::vector<Vertex<T> *> validVertices;

    toVisit.push(pStartVertex);
    seenVertices.insert(pStartVertex);

    do {
      Vertex<T> * pNext = toVisit.front();
      toVisit.pop();

      if (pNext != NULL && pVisitor->Visit(pNext)) {
        // vertex is valid, explore neighbors
        validVertices.push_back(pNext);

        std::vector<Vertex<T> *> adjacentVertices = pNext->GetAdjacentVertices();
        forEach(typename std::vector<Vertex<T> *>, &adjacentVertices)
        {
          Vertex<T> * pAdjacent = *iter;

          // adjacent vertex has not yet been seen, add to queue for processing
          if (seenVertices.find(pAdjacent) == seenVertices.end()) {
            toVisit.push(pAdjacent);
            seenVertices.insert(pAdjacent);
          }
        }
      }
    } while (toVisit.empty() == false);

    return validVertices;
  }

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GraphTraversal<T>);
  }
};    // class BreadthFirstTraversal

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class NearScanVisitor : public Visitor<LocalizedRangeScan>
{
public:
  NearScanVisitor(LocalizedRangeScan * pScan, kt_double maxDistance, kt_bool useScanBarycenter)
  : m_MaxDistanceSquared(math::Square(maxDistance)),
    m_UseScanBarycenter(useScanBarycenter)
  {
    m_CenterPose = pScan->GetReferencePose(m_UseScanBarycenter);
  }

  virtual kt_bool Visit(Vertex<LocalizedRangeScan> * pVertex)
  {
    try {
      LocalizedRangeScan * pScan = pVertex->GetObject();
      Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);
      kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
      return squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE;
    } catch (...) {
      // relocalization vertex elements missing
      std::cout << "Unable to visit valid vertex elements!" << std::endl;
      return false;
    }
  }

protected:
  Pose2 m_CenterPose;
  kt_double m_MaxDistanceSquared;
  kt_bool m_UseScanBarycenter;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Visitor<LocalizedRangeScan>);
    ar & BOOST_SERIALIZATION_NVP(m_CenterPose);
    ar & BOOST_SERIALIZATION_NVP(m_MaxDistanceSquared);
    ar & BOOST_SERIALIZATION_NVP(m_UseScanBarycenter);
  }
};    // NearScanVisitor

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

class NearPoseVisitor : public Visitor<LocalizedRangeScan>
{
public:
  NearPoseVisitor(Pose2 refPose, kt_double maxDistance, kt_bool useScanBarycenter)
  : m_MaxDistanceSquared(math::Square(maxDistance)),
    m_UseScanBarycenter(useScanBarycenter)
  {
    m_CenterPose = refPose;
  }

  virtual kt_bool Visit(Vertex<LocalizedRangeScan> * pVertex)
  {
    LocalizedRangeScan * pScan = pVertex->GetObject();

    Pose2 pose = pScan->GetReferencePose(m_UseScanBarycenter);

    kt_double squaredDistance = pose.GetPosition().SquaredDistance(m_CenterPose.GetPosition());
    return squaredDistance <= m_MaxDistanceSquared - KT_TOLERANCE;
  }

protected:
  Pose2 m_CenterPose;
  kt_double m_MaxDistanceSquared;
  kt_bool m_UseScanBarycenter;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Visitor<LocalizedRangeScan>);
    ar & BOOST_SERIALIZATION_NVP(m_CenterPose);
    ar & BOOST_SERIALIZATION_NVP(m_MaxDistanceSquared);
    ar & BOOST_SERIALIZATION_NVP(m_UseScanBarycenter);
  }
};    // NearPoseVisitor

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


MapperGraph::MapperGraph(Mapper * pMapper, kt_double rangeThreshold)
: m_pMapper(pMapper)
{
  m_pLoopScanMatcher = ScanMatcher::Create(pMapper,
    m_pMapper->m_pLoopSearchSpaceDimension->GetValue(),
    m_pMapper->m_pLoopSearchSpaceResolution->GetValue(),
    m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
  assert(m_pLoopScanMatcher);

  m_pTraversal = new BreadthFirstTraversal<LocalizedRangeScan>(this);
}

MapperGraph::~MapperGraph()
{
  if (m_pLoopScanMatcher) {
    delete m_pLoopScanMatcher;
    m_pLoopScanMatcher = NULL;
  }
  if (m_pTraversal) {
    delete m_pTraversal;
    m_pTraversal = NULL;
  }
}

Vertex<LocalizedRangeScan> * MapperGraph::AddVertex(LocalizedRangeScan * pScan)
{
  assert(pScan);

  if (pScan != NULL) {
    Vertex<LocalizedRangeScan> * pVertex = new Vertex<LocalizedRangeScan>(pScan);
    Graph<LocalizedRangeScan>::AddVertex(pScan->GetSensorName(), pVertex);
    if (m_pMapper->m_pScanOptimizer != NULL) {
      m_pMapper->m_pScanOptimizer->AddNode(pVertex);
    }
    return pVertex;
  }

  return nullptr;
}

void MapperGraph::AddEdges(LocalizedRangeScan * pScan, const Matrix3 & rCovariance)
{
  MapperSensorManager * pSensorManager = m_pMapper->m_pMapperSensorManager;

  const Name rSensorName = pScan->GetSensorName();

  // link to previous scan
  kt_int32s previousScanNum = pScan->GetStateId() - 1;
  if (pSensorManager->GetLastScan(rSensorName) != NULL) {
    assert(previousScanNum >= 0);
    LocalizedRangeScan * pPrevScan = pSensorManager->GetScan(rSensorName, previousScanNum);
    if (!pPrevScan) {
      return;
    }
    LinkScans(pPrevScan, pScan, pScan->GetSensorPose(), rCovariance);
  }

  Pose2Vector means;
  std::vector<Matrix3> covariances;

  // first scan (link to first scan of other robots)
  if (pSensorManager->GetLastScan(rSensorName) == NULL) {
    assert(pSensorManager->GetScans(rSensorName).size() == 1);

    std::vector<Name> deviceNames = pSensorManager->GetSensorNames();
    forEach(std::vector<Name>, &deviceNames)
    {
      const Name & rCandidateSensorName = *iter;

      // skip if candidate device is the same or other device has no scans
      if ((rCandidateSensorName == rSensorName) ||
        (pSensorManager->GetScans(rCandidateSensorName).empty()))
      {
        continue;
      }

      Pose2 bestPose;
      Matrix3 covariance;
      kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan<LocalizedRangeScanMap>(
        pScan,
        pSensorManager->GetScans(rCandidateSensorName),
        bestPose, covariance);
      LinkScans(pSensorManager->GetScan(rCandidateSensorName, 0), pScan, bestPose, covariance);

      // only add to means and covariances if response was high "enough"
      if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue()) {
        means.push_back(bestPose);
        covariances.push_back(covariance);
      }
    }
  } else {
    // link to running scans
    Pose2 scanPose = pScan->GetSensorPose();
    means.push_back(scanPose);
    covariances.push_back(rCovariance);
    LinkChainToScan(pSensorManager->GetRunningScans(rSensorName), pScan, scanPose, rCovariance);
  }

  // link to other near chains (chains that include new scan are invalid)
  LinkNearChains(pScan, means, covariances);

  if (!means.empty()) {
    pScan->SetSensorPose(ComputeWeightedMean(means, covariances));
  }
}

kt_bool MapperGraph::TryCloseLoop(LocalizedRangeScan * pScan, const Name & rSensorName)
{
  kt_bool loopClosed = false;

  kt_int32u scanIndex = 0;

  LocalizedRangeScanVector candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);

  while (!candidateChain.empty()) {
    Pose2 bestPose;
    Matrix3 covariance;
    kt_double coarseResponse = m_pLoopScanMatcher->MatchScan(pScan, candidateChain,
        bestPose, covariance, false, false);

    std::stringstream stream;
    stream << "COARSE RESPONSE: " << coarseResponse <<
      " (> " << m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue() << ")" <<
      std::endl;
    stream << "            var: " << covariance(0, 0) << ",  " << covariance(1, 1) <<
      " (< " << m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue() << ")";

    m_pMapper->FireLoopClosureCheck(stream.str());

    if ((coarseResponse > m_pMapper->m_pLoopMatchMinimumResponseCoarse->GetValue()) &&
      (covariance(0, 0) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()) &&
      (covariance(1, 1) < m_pMapper->m_pLoopMatchMaximumVarianceCoarse->GetValue()))
    {
      LocalizedRangeScan tmpScan(pScan->GetSensorName(), pScan->GetRangeReadingsVector());
      tmpScan.SetUniqueId(pScan->GetUniqueId());
      tmpScan.SetTime(pScan->GetTime());
      tmpScan.SetStateId(pScan->GetStateId());
      tmpScan.SetCorrectedPose(pScan->GetCorrectedPose());
      tmpScan.SetSensorPose(bestPose);    // This also updates OdometricPose.
      kt_double fineResponse = m_pMapper->m_pSequentialScanMatcher->MatchScan(&tmpScan,
          candidateChain,
          bestPose, covariance, false);

      std::stringstream stream1;
      stream1 << "FINE RESPONSE: " << fineResponse << " (>" <<
        m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue() << ")" << std::endl;
      m_pMapper->FireLoopClosureCheck(stream1.str());

      if (fineResponse < m_pMapper->m_pLoopMatchMinimumResponseFine->GetValue()) {
        m_pMapper->FireLoopClosureCheck("REJECTED!");
      } else {
        m_pMapper->FireBeginLoopClosure("Closing loop...");

        pScan->SetSensorPose(bestPose);
        LinkChainToScan(candidateChain, pScan, bestPose, covariance);
        CorrectPoses();

        m_pMapper->FireEndLoopClosure("Loop closed!");

        loopClosed = true;
      }
    }

    candidateChain = FindPossibleLoopClosure(pScan, rSensorName, scanIndex);
  }

  return loopClosed;
}

LocalizedRangeScan * MapperGraph::GetClosestScanToPose(
  const LocalizedRangeScanVector & rScans,
  const Pose2 & rPose) const
{
  LocalizedRangeScan * pClosestScan = NULL;
  kt_double bestSquaredDistance = DBL_MAX;

  const_forEach(LocalizedRangeScanVector, &rScans)
  {
    Pose2 scanPose = (*iter)->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = rPose.GetPosition().SquaredDistance(scanPose.GetPosition());
    if (squaredDistance < bestSquaredDistance) {
      bestSquaredDistance = squaredDistance;
      pClosestScan = *iter;
    }
  }

  return pClosestScan;
}

Edge<LocalizedRangeScan> * MapperGraph::AddEdge(
  LocalizedRangeScan * pSourceScan,
  LocalizedRangeScan * pTargetScan, kt_bool & rIsNewEdge)
{
  std::map<int,
    Vertex<LocalizedRangeScan> *>::iterator v1 = m_Vertices[pSourceScan->GetSensorName()].find(
    pSourceScan->GetStateId());
  std::map<int,
    Vertex<LocalizedRangeScan> *>::iterator v2 = m_Vertices[pTargetScan->GetSensorName()].find(
    pTargetScan->GetStateId());

  if (v1 == m_Vertices[pSourceScan->GetSensorName()].end() ||
    v2 == m_Vertices[pSourceScan->GetSensorName()].end())
  {
    std::cout << "AddEdge: At least one vertex is invalid." << std::endl;
    return NULL;
  }

  // see if edge already exists
  const_forEach(std::vector<Edge<LocalizedRangeScan> *>, &(v1->second->GetEdges()))
  {
    Edge<LocalizedRangeScan> * pEdge = *iter;

    if (pEdge->GetTarget() == v2->second) {
      rIsNewEdge = false;
      return pEdge;
    }
  }

  Edge<LocalizedRangeScan> * pEdge = new Edge<LocalizedRangeScan>(v1->second, v2->second);
  Graph<LocalizedRangeScan>::AddEdge(pEdge);
  rIsNewEdge = true;
  return pEdge;
}

void MapperGraph::LinkScans(
  LocalizedRangeScan * pFromScan, LocalizedRangeScan * pToScan,
  const Pose2 & rMean, const Matrix3 & rCovariance)
{
  kt_bool isNewEdge = true;
  Edge<LocalizedRangeScan> * pEdge = AddEdge(pFromScan, pToScan, isNewEdge);

  if (pEdge == NULL) {
    return;
  }

  // only attach link information if the edge is new
  if (isNewEdge == true) {
    pEdge->SetLabel(new LinkInfo(pFromScan->GetCorrectedPose(), pToScan->GetCorrectedAt(rMean), rCovariance));
    if (m_pMapper->m_pScanOptimizer != NULL) {
      m_pMapper->m_pScanOptimizer->AddConstraint(pEdge);
    }
  }
}

void MapperGraph::LinkNearChains(
  LocalizedRangeScan * pScan, Pose2Vector & rMeans,
  std::vector<Matrix3> & rCovariances)
{
  const std::vector<LocalizedRangeScanVector> nearChains = FindNearChains(pScan);
  const_forEach(std::vector<LocalizedRangeScanVector>, &nearChains)
  {
    if (iter->size() < m_pMapper->m_pLoopMatchMinimumChainSize->GetValue()) {
      continue;
    }

    Pose2 mean;
    Matrix3 covariance;
    // match scan against "near" chain
    kt_double response = m_pMapper->m_pSequentialScanMatcher->MatchScan(pScan, *iter, mean,
        covariance, false);
    if (response > m_pMapper->m_pLinkMatchMinimumResponseFine->GetValue() - KT_TOLERANCE) {
      rMeans.push_back(mean);
      rCovariances.push_back(covariance);
      LinkChainToScan(*iter, pScan, mean, covariance);
    }
  }
}

void MapperGraph::LinkChainToScan(
  const LocalizedRangeScanVector & rChain, LocalizedRangeScan * pScan,
  const Pose2 & rMean, const Matrix3 & rCovariance)
{
  Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  LocalizedRangeScan * pClosestScan = GetClosestScanToPose(rChain, pose);
  assert(pClosestScan != NULL);

  Pose2 closestScanPose =
    pClosestScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  kt_double squaredDistance = pose.GetPosition().SquaredDistance(closestScanPose.GetPosition());
  if (squaredDistance <
    math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
  {
    LinkScans(pClosestScan, pScan, rMean, rCovariance);
  }
}

std::vector<LocalizedRangeScanVector> MapperGraph::FindNearChains(LocalizedRangeScan * pScan)
{
  std::vector<LocalizedRangeScanVector> nearChains;

  Pose2 scanPose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  // to keep track of which scans have been added to a chain
  LocalizedRangeScanVector processed;

  const LocalizedRangeScanVector nearLinkedScans = FindNearLinkedScans(pScan,
      m_pMapper->m_pLinkScanMaximumDistance->GetValue());
  const_forEach(LocalizedRangeScanVector, &nearLinkedScans)
  {
    LocalizedRangeScan * pNearScan = *iter;

    if (pNearScan == pScan) {
      continue;
    }

    // scan has already been processed, skip
    if (find(processed.begin(), processed.end(), pNearScan) != processed.end()) {
      continue;
    }

    processed.push_back(pNearScan);

    // build up chain
    kt_bool isValidChain = true;
    std::list<LocalizedRangeScan *> chain;

    // add scans before current scan being processed
    for (kt_int32s candidateScanNum = pNearScan->GetStateId() - 1; candidateScanNum >= 0;
      candidateScanNum--)
    {
      LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(
        pNearScan->GetSensorName(),
        candidateScanNum);

      // chain is invalid--contains scan being added
      if (pCandidateScan == pScan) {
        isValidChain = false;
      }

      // probably removed in localization mode
      if (pCandidateScan == NULL) {
        continue;
      }

      Pose2 candidatePose = pCandidateScan->GetReferencePose(
        m_pMapper->m_pUseScanBarycenter->GetValue());
      kt_double squaredDistance =
        scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

      if (squaredDistance <
        math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        chain.push_front(pCandidateScan);
        processed.push_back(pCandidateScan);
      } else {
        break;
      }
    }

    chain.push_back(pNearScan);

    // add scans after current scan being processed
    kt_int32u end =
      static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(
        pNearScan->GetSensorName()).size());
    for (kt_int32u candidateScanNum = pNearScan->GetStateId() + 1; candidateScanNum < end;
      candidateScanNum++)
    {
      LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(
        pNearScan->GetSensorName(),
        candidateScanNum);

      if (pCandidateScan == pScan) {
        isValidChain = false;
      }

      // probably removed in localization mode
      if (pCandidateScan == NULL) {
        continue;
      }

      Pose2 candidatePose = pCandidateScan->GetReferencePose(
        m_pMapper->m_pUseScanBarycenter->GetValue());
      kt_double squaredDistance =
        scanPose.GetPosition().SquaredDistance(candidatePose.GetPosition());

      if (squaredDistance <
        math::Square(m_pMapper->m_pLinkScanMaximumDistance->GetValue()) + KT_TOLERANCE)
      {
        chain.push_back(pCandidateScan);
        processed.push_back(pCandidateScan);
      } else {
        break;
      }
    }

    if (isValidChain) {
      // change list to vector
      LocalizedRangeScanVector tempChain;
      std::copy(chain.begin(), chain.end(), std::inserter(tempChain, tempChain.begin()));
      // add chain to collection
      nearChains.push_back(tempChain);
    }
  }

  return nearChains;
}

LocalizedRangeScanVector MapperGraph::FindNearLinkedScans(
  LocalizedRangeScan * pScan,
  kt_double maxDistance)
{
  NearScanVisitor * pVisitor = new NearScanVisitor(pScan, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());
  LocalizedRangeScanVector nearLinkedScans = m_pTraversal->TraverseForScans(GetVertex(
        pScan), pVisitor);
  delete pVisitor;

  return nearLinkedScans;
}

std::vector<Vertex<LocalizedRangeScan> *> MapperGraph::FindNearLinkedVertices(
  LocalizedRangeScan * pScan, kt_double maxDistance)
{
  NearScanVisitor * pVisitor = new NearScanVisitor(pScan, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());
  std::vector<Vertex<LocalizedRangeScan> *> nearLinkedVertices =
    m_pTraversal->TraverseForVertices(GetVertex(
        pScan), pVisitor);
  delete pVisitor;

  return nearLinkedVertices;
}

LocalizedRangeScanVector MapperGraph::FindNearByScans(
  Name name, const Pose2 refPose,
  kt_double maxDistance)
{
  NearPoseVisitor * pVisitor = new NearPoseVisitor(refPose, maxDistance,
      m_pMapper->m_pUseScanBarycenter->GetValue());

  Vertex<LocalizedRangeScan> * closestVertex = FindNearByScan(name, refPose);

  LocalizedRangeScanVector nearLinkedScans =
    m_pTraversal->TraverseForScans(closestVertex, pVisitor);
  delete pVisitor;

  return nearLinkedScans;
}

std::vector<Vertex<LocalizedRangeScan> *> MapperGraph::FindNearByVertices(
  Name name,
  const Pose2 refPose,
  kt_double maxDistance)
{
  VertexMap vertexMap = GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> & vertices = vertexMap[name];

  std::vector<Vertex<LocalizedRangeScan> *> vertices_to_search;
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator it;
  for (it = vertices.begin(); it != vertices.end(); ++it) {
    if (it->second) {
      vertices_to_search.push_back(it->second);
    }
  }

  const size_t dim = 2;

  typedef VertexVectorPoseNanoFlannAdaptor<std::vector<Vertex<LocalizedRangeScan> *>> P2KD;
  const P2KD p2kd(vertices_to_search);

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, P2KD>, P2KD,
      dim> my_kd_tree_t;

  my_kd_tree_t index(dim, p2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
  index.buildIndex();

  std::vector<std::pair<size_t, double>> ret_matches;
  const double query_pt[2] = {refPose.GetX(), refPose.GetY()};
  nanoflann::SearchParams params;
  const size_t num_results = index.radiusSearch(&query_pt[0], maxDistance, ret_matches, params);

  std::vector<Vertex<LocalizedRangeScan> *> rtn_vertices;
  rtn_vertices.reserve(ret_matches.size());
  for (uint i = 0; i != ret_matches.size(); i++) {
    rtn_vertices.push_back(vertices_to_search[ret_matches[i].first]);
  }
  return rtn_vertices;
}

Vertex<LocalizedRangeScan> * MapperGraph::FindNearByScan(Name name, const Pose2 refPose)
{
  VertexMap vertexMap = GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> & vertices = vertexMap[name];

  std::vector<Vertex<LocalizedRangeScan> *> vertices_to_search;
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator it;
  for (it = vertices.begin(); it != vertices.end(); ++it) {
    if (it->second) {
      vertices_to_search.push_back(it->second);
    }
  }

  size_t num_results = 1;
  const size_t dim = 2;

  typedef VertexVectorPoseNanoFlannAdaptor<std::vector<Vertex<LocalizedRangeScan> *>> P2KD;
  const P2KD p2kd(vertices_to_search);

  typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, P2KD>, P2KD,
      dim> my_kd_tree_t;

  my_kd_tree_t index(dim, p2kd, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
  index.buildIndex();

  std::vector<size_t> ret_index(num_results);
  std::vector<double> out_dist_sqr(num_results);
  const double query_pt[2] = {refPose.GetX(), refPose.GetY()};
  num_results = index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  if (num_results > 0) {
    return vertices_to_search[ret_index[0]];
  } else {
    return NULL;
  }
}

Pose2 MapperGraph::ComputeWeightedMean(
  const Pose2Vector & rMeans,
  const std::vector<Matrix3> & rCovariances) const
{
  assert(rMeans.size() == rCovariances.size());

  // compute sum of inverses and create inverse list
  std::vector<Matrix3> inverses;
  inverses.reserve(rCovariances.size());

  Matrix3 sumOfInverses;
  const_forEach(std::vector<Matrix3>, &rCovariances)
  {
    Matrix3 inverse = iter->Inverse();
    inverses.push_back(inverse);

    sumOfInverses += inverse;
  }
  Matrix3 inverseOfSumOfInverses = sumOfInverses.Inverse();

  // compute weighted mean
  Pose2 accumulatedPose;
  kt_double thetaX = 0.0;
  kt_double thetaY = 0.0;

  Pose2Vector::const_iterator meansIter = rMeans.begin();
  const_forEach(std::vector<Matrix3>, &inverses)
  {
    Pose2 pose = *meansIter;
    kt_double angle = pose.GetHeading();
    thetaX += cos(angle);
    thetaY += sin(angle);

    Matrix3 weight = inverseOfSumOfInverses * (*iter);
    accumulatedPose += weight * pose;

    ++meansIter;
  }

  thetaX /= rMeans.size();
  thetaY /= rMeans.size();
  accumulatedPose.SetHeading(atan2(thetaY, thetaX));

  return accumulatedPose;
}

LocalizedRangeScanVector MapperGraph::FindPossibleLoopClosure(
  LocalizedRangeScan * pScan,
  const Name & rSensorName,
  kt_int32u & rStartNum)
{
  LocalizedRangeScanVector chain;    // return value

  Pose2 pose = pScan->GetReferencePose(m_pMapper->m_pUseScanBarycenter->GetValue());

  // possible loop closure chain should not include close scans that have a
  // path of links to the scan of interest
  const LocalizedRangeScanVector nearLinkedScans =
    FindNearLinkedScans(pScan, m_pMapper->m_pLoopSearchMaximumDistance->GetValue());

  kt_int32u nScans =
    static_cast<kt_int32u>(m_pMapper->m_pMapperSensorManager->GetScans(rSensorName).size());
  for (; rStartNum < nScans; rStartNum++) {
    LocalizedRangeScan * pCandidateScan = m_pMapper->m_pMapperSensorManager->GetScan(rSensorName,
        rStartNum);

    if (pCandidateScan == NULL) {
      continue;
    }

    Pose2 candidateScanPose = pCandidateScan->GetReferencePose(
      m_pMapper->m_pUseScanBarycenter->GetValue());

    kt_double squaredDistance = candidateScanPose.GetPosition().SquaredDistance(pose.GetPosition());
    if (squaredDistance <
      math::Square(m_pMapper->m_pLoopSearchMaximumDistance->GetValue()) + KT_TOLERANCE)
    {
      // a linked scan cannot be in the chain
      if (find(nearLinkedScans.begin(), nearLinkedScans.end(),
        pCandidateScan) != nearLinkedScans.end())
      {
        chain.clear();
      } else {
        chain.push_back(pCandidateScan);
      }
    } else {
      // return chain if it is long "enough"
      if (chain.size() >= m_pMapper->m_pLoopMatchMinimumChainSize->GetValue()) {
        return chain;
      } else {
        chain.clear();
      }
    }
  }

  return chain;
}

void MapperGraph::CorrectPoses()
{
  // optimize scans!
  ScanSolver * pSolver = m_pMapper->m_pScanOptimizer;
  if (pSolver != NULL) {
    pSolver->Compute();

    const_forEach(ScanSolver::IdPoseVector, &pSolver->GetCorrections())
    {
      LocalizedRangeScan * scan = m_pMapper->m_pMapperSensorManager->GetScan(iter->first);
      if (scan == NULL) {
        continue;
      }
      scan->SetCorrectedPoseAndUpdate(iter->second);
    }

    pSolver->Clear();
  }
}

void MapperGraph::UpdateLoopScanMatcher(kt_double rangeThreshold)
{
  if (m_pLoopScanMatcher) {
    delete m_pLoopScanMatcher;
  }
  m_pLoopScanMatcher = ScanMatcher::Create(m_pMapper,
    m_pMapper->m_pLoopSearchSpaceDimension->GetValue(),
    m_pMapper->m_pLoopSearchSpaceResolution->GetValue(),
    m_pMapper->m_pLoopSearchSpaceSmearDeviation->GetValue(), rangeThreshold);
  assert(m_pLoopScanMatcher);
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Default constructor
 */
Mapper::Mapper()
: Module("Mapper"),
  m_Initialized(false),
  m_Deserialized(false),
  m_pSequentialScanMatcher(NULL),
  m_pMapperSensorManager(NULL),
  m_pGraph(NULL),
  m_pScanOptimizer(NULL)
{
  InitializeParameters();
}

/**
 * Default constructor
 */
Mapper::Mapper(const std::string & rName)
: Module(rName),
  m_Initialized(false),
  m_Deserialized(false),
  m_pSequentialScanMatcher(NULL),
  m_pMapperSensorManager(NULL),
  m_pGraph(NULL),
  m_pScanOptimizer(NULL)
{
  InitializeParameters();
}

/**
 * Destructor
 */
Mapper::~Mapper()
{
  Reset();

  delete m_pMapperSensorManager;
}

void Mapper::InitializeParameters()
{
  m_pUseScanMatching = new Parameter<kt_bool>(
    "UseScanMatching",
    "When set to true, the mapper will use a scan matching algorithm. "
    "In most real-world situations this should be set to true so that the "
    "mapper algorithm can correct for noise and errors in odometry and "
    "scan data. In some simulator environments where the simulated scan "
    "and odometry data are very accurate, the scan matching algorithm can "
    "produce worse results. In those cases set this to false to improve "
    "results.",
    true,
    GetParameterManager());

  m_pUseScanBarycenter = new Parameter<kt_bool>(
    "UseScanBarycenter",
    "Use the barycenter of scan endpoints to define distances between "
    "scans.",
    true, GetParameterManager());

  m_pMinimumTimeInterval = new Parameter<kt_double>(
    "MinimumTimeInterval",
    "Sets the minimum time between scans. If a new scan's time stamp is "
    "longer than MinimumTimeInterval from the previously processed scan, "
    "the mapper will use the data from the new scan. Otherwise, it will "
    "discard the new scan if it also does not meet the minimum travel "
    "distance and heading requirements. For performance reasons, it is "
    "generally it is a good idea to only process scans if a reasonable "
    "amount of time has passed. This parameter is particularly useful "
    "when there is a need to process scans while the robot is stationary.",
    3600, GetParameterManager());

  m_pMinimumTravelDistance = new Parameter<kt_double>(
    "MinimumTravelDistance",
    "Sets the minimum travel between scans.  If a new scan's position is "
    "more than minimumTravelDistance from the previous scan, the mapper "
    "will use the data from the new scan. Otherwise, it will discard the "
    "new scan if it also does not meet the minimum change in heading "
    "requirement. For performance reasons, generally it is a good idea to "
    "only process scans if the robot has moved a reasonable amount.",
    0.2, GetParameterManager());

  m_pMinimumTravelHeading = new Parameter<kt_double>(
    "MinimumTravelHeading",
    "Sets the minimum heading change between scans. If a new scan's "
    "heading is more than MinimumTravelHeading from the previous scan, the "
    "mapper will use the data from the new scan.  Otherwise, it will "
    "discard the new scan if it also does not meet the minimum travel "
    "distance requirement. For performance reasons, generally it is a good "
    "idea to only process scans if the robot has moved a reasonable "
    "amount.",
    math::DegreesToRadians(10), GetParameterManager());

  m_pScanBufferSize = new Parameter<kt_int32u>(
    "ScanBufferSize",
    "Scan buffer size is the length of the scan chain stored for scan "
    "matching. \"ScanBufferSize\" should be set to approximately "
    "\"ScanBufferMaximumScanDistance\" / \"MinimumTravelDistance\". The "
    "idea is to get an area approximately 20 meters long for scan "
    "matching. For example, if we add scans every MinimumTravelDistance == "
    "0.3 meters, then \"scanBufferSize\" should be 20 / 0.3 = 67.)",
    70, GetParameterManager());

  m_pScanBufferMaximumScanDistance = new Parameter<kt_double>(
    "ScanBufferMaximumScanDistance",
    "Scan buffer maximum scan distance is the maximum distance between the "
    "first and last scans in the scan chain stored for matching.",
    20.0, GetParameterManager());

  m_pLinkMatchMinimumResponseFine = new Parameter<kt_double>(
    "LinkMatchMinimumResponseFine",
    "Scans are linked only if the correlation response value is greater "
    "than this value.",
    0.8, GetParameterManager());

  m_pLinkScanMaximumDistance = new Parameter<kt_double>(
    "LinkScanMaximumDistance",
    "Maximum distance between linked scans.  Scans that are farther apart "
    "will not be linked regardless of the correlation response value.",
    10.0, GetParameterManager());

  m_pLoopSearchMaximumDistance = new Parameter<kt_double>(
    "LoopSearchMaximumDistance",
    "Scans less than this distance from the current position will be "
    "considered for a match in loop closure.",
    4.0, GetParameterManager());

  m_pDoLoopClosing = new Parameter<kt_bool>(
    "DoLoopClosing",
    "Enable/disable loop closure.",
    true, GetParameterManager());

  m_pLoopMatchMinimumChainSize = new Parameter<kt_int32u>(
    "LoopMatchMinimumChainSize",
    "When the loop closure detection finds a candidate it must be part of "
    "a large set of linked scans. If the chain of scans is less than this "
    "value we do not attempt to close the loop.",
    10, GetParameterManager());

  m_pLoopMatchMaximumVarianceCoarse = new Parameter<kt_double>(
    "LoopMatchMaximumVarianceCoarse",
    "The co-variance values for a possible loop closure have to be less "
    "than this value to consider a viable solution. This applies to the "
    "coarse search.",
    math::Square(0.4), GetParameterManager());

  m_pLoopMatchMinimumResponseCoarse = new Parameter<kt_double>(
    "LoopMatchMinimumResponseCoarse",
    "If response is larger then this, then initiate loop closure search at "
    "the coarse resolution.",
    0.8, GetParameterManager());

  m_pLoopMatchMinimumResponseFine = new Parameter<kt_double>(
    "LoopMatchMinimumResponseFine",
    "If response is larger then this, then initiate loop closure search at "
    "the fine resolution.",
    0.8, GetParameterManager());

  //////////////////////////////////////////////////////////////////////////////
  //    CorrelationParameters correlationParameters;

  m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>(
    "CorrelationSearchSpaceDimension",
    "The size of the search grid used by the matcher. The search grid will "
    "have the size CorrelationSearchSpaceDimension * "
    "CorrelationSearchSpaceDimension",
    0.3, GetParameterManager());

  m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>(
    "CorrelationSearchSpaceResolution",
    "The resolution (size of a grid cell) of the correlation grid.",
    0.01, GetParameterManager());

  m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>(
    "CorrelationSearchSpaceSmearDeviation",
    "The point readings are smeared by this value in X and Y to create a "
    "smoother response.",
    0.03, GetParameterManager());


  //////////////////////////////////////////////////////////////////////////////
  //    CorrelationParameters loopCorrelationParameters;

  m_pLoopSearchSpaceDimension = new Parameter<kt_double>(
    "LoopSearchSpaceDimension",
    "The size of the search grid used by the matcher.",
    8.0, GetParameterManager());

  m_pLoopSearchSpaceResolution = new Parameter<kt_double>(
    "LoopSearchSpaceResolution",
    "The resolution (size of a grid cell) of the correlation grid.",
    0.05, GetParameterManager());

  m_pLoopSearchSpaceSmearDeviation = new Parameter<kt_double>(
    "LoopSearchSpaceSmearDeviation",
    "The point readings are smeared by this value in X and Y to create a "
    "smoother response.",
    0.03, GetParameterManager());

  //////////////////////////////////////////////////////////////////////////////
  // ScanMatcherParameters;

  m_pDistanceVariancePenalty = new Parameter<kt_double>(
    "DistanceVariancePenalty",
    "Variance of penalty for deviating from odometry when scan-matching. "
    "The penalty is a multiplier (less than 1.0) is a function of the "
    "delta of the scan position being tested and the odometric pose.",
    math::Square(0.3), GetParameterManager());

  m_pAngleVariancePenalty = new Parameter<kt_double>(
    "AngleVariancePenalty",
    "See DistanceVariancePenalty.",
    math::Square(math::DegreesToRadians(20)), GetParameterManager());

  m_pFineSearchAngleOffset = new Parameter<kt_double>(
    "FineSearchAngleOffset",
    "The range of angles to search during a fine search.",
    math::DegreesToRadians(0.2), GetParameterManager());

  m_pCoarseSearchAngleOffset = new Parameter<kt_double>(
    "CoarseSearchAngleOffset",
    "The range of angles to search during a coarse search.",
    math::DegreesToRadians(20), GetParameterManager());

  m_pCoarseAngleResolution = new Parameter<kt_double>(
    "CoarseAngleResolution",
    "Resolution of angles to search during a coarse search.",
    math::DegreesToRadians(2), GetParameterManager());

  m_pMinimumAnglePenalty = new Parameter<kt_double>(
    "MinimumAnglePenalty",
    "Minimum value of the angle penalty multiplier so scores do not become "
    "too small.",
    0.9, GetParameterManager());

  m_pMinimumDistancePenalty = new Parameter<kt_double>(
    "MinimumDistancePenalty",
    "Minimum value of the distance penalty multiplier so scores do not "
    "become too small.",
    0.5, GetParameterManager());

  m_pUseResponseExpansion = new Parameter<kt_bool>(
    "UseResponseExpansion",
    "Whether to increase the search space if no good matches are initially "
    "found.",
    false, GetParameterManager());

  m_pMinPassThrough = new Parameter<kt_int32u>(
    "MinPassThrough",
    "Number of beams that must pass through a cell before it will be considered to be occupied "
    "or unoccupied.  This prevents stray beams from messing up the map. "
    "found.",
    2, GetParameterManager());

  m_pOccupancyThreshold = new Parameter<kt_double>(
    "OccupancyThreshold",
    "Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied",
    0.1, GetParameterManager());
}
/* Adding in getters and setters here for easy parameter access */

// General Parameters

bool Mapper::getParamUseScanMatching()
{
  return static_cast<bool>(m_pUseScanMatching->GetValue());
}

bool Mapper::getParamUseScanBarycenter()
{
  return static_cast<bool>(m_pUseScanBarycenter->GetValue());
}

double Mapper::getParamMinimumTimeInterval()
{
  return static_cast<double>(m_pMinimumTimeInterval->GetValue());
}

double Mapper::getParamMinimumTravelDistance()
{
  return static_cast<double>(m_pMinimumTravelDistance->GetValue());
}

double Mapper::getParamMinimumTravelHeading()
{
  return math::RadiansToDegrees(static_cast<double>(m_pMinimumTravelHeading->GetValue()));
}

int Mapper::getParamScanBufferSize()
{
  return static_cast<int>(m_pScanBufferSize->GetValue());
}

double Mapper::getParamScanBufferMaximumScanDistance()
{
  return static_cast<double>(m_pScanBufferMaximumScanDistance->GetValue());
}

double Mapper::getParamLinkMatchMinimumResponseFine()
{
  return static_cast<double>(m_pLinkMatchMinimumResponseFine->GetValue());
}

double Mapper::getParamLinkScanMaximumDistance()
{
  return static_cast<double>(m_pLinkScanMaximumDistance->GetValue());
}

double Mapper::getParamLoopSearchMaximumDistance()
{
  return static_cast<double>(m_pLoopSearchMaximumDistance->GetValue());
}

bool Mapper::getParamDoLoopClosing()
{
  return static_cast<bool>(m_pDoLoopClosing->GetValue());
}

int Mapper::getParamLoopMatchMinimumChainSize()
{
  return static_cast<int>(m_pLoopMatchMinimumChainSize->GetValue());
}

double Mapper::getParamLoopMatchMaximumVarianceCoarse()
{
  return static_cast<double>(std::sqrt(m_pLoopMatchMaximumVarianceCoarse->GetValue()));
}

double Mapper::getParamLoopMatchMinimumResponseCoarse()
{
  return static_cast<double>(m_pLoopMatchMinimumResponseCoarse->GetValue());
}

double Mapper::getParamLoopMatchMinimumResponseFine()
{
  return static_cast<double>(m_pLoopMatchMinimumResponseFine->GetValue());
}

// Correlation Parameters - Correlation Parameters

double Mapper::getParamCorrelationSearchSpaceDimension()
{
  return static_cast<double>(m_pCorrelationSearchSpaceDimension->GetValue());
}

double Mapper::getParamCorrelationSearchSpaceResolution()
{
  return static_cast<double>(m_pCorrelationSearchSpaceResolution->GetValue());
}

double Mapper::getParamCorrelationSearchSpaceSmearDeviation()
{
  return static_cast<double>(m_pCorrelationSearchSpaceSmearDeviation->GetValue());
}

// Correlation Parameters - Loop Correlation Parameters

double Mapper::getParamLoopSearchSpaceDimension()
{
  return static_cast<double>(m_pLoopSearchSpaceDimension->GetValue());
}

double Mapper::getParamLoopSearchSpaceResolution()
{
  return static_cast<double>(m_pLoopSearchSpaceResolution->GetValue());
}

double Mapper::getParamLoopSearchSpaceSmearDeviation()
{
  return static_cast<double>(m_pLoopSearchSpaceSmearDeviation->GetValue());
}

// ScanMatcher Parameters

double Mapper::getParamDistanceVariancePenalty()
{
  return std::sqrt(static_cast<double>(m_pDistanceVariancePenalty->GetValue()));
}

double Mapper::getParamAngleVariancePenalty()
{
  return std::sqrt(static_cast<double>(m_pAngleVariancePenalty->GetValue()));
}

double Mapper::getParamFineSearchAngleOffset()
{
  return static_cast<double>(m_pFineSearchAngleOffset->GetValue());
}

double Mapper::getParamCoarseSearchAngleOffset()
{
  return static_cast<double>(m_pCoarseSearchAngleOffset->GetValue());
}

double Mapper::getParamCoarseAngleResolution()
{
  return static_cast<double>(m_pCoarseAngleResolution->GetValue());
}

double Mapper::getParamMinimumAnglePenalty()
{
  return static_cast<double>(m_pMinimumAnglePenalty->GetValue());
}

double Mapper::getParamMinimumDistancePenalty()
{
  return static_cast<double>(m_pMinimumDistancePenalty->GetValue());
}

bool Mapper::getParamUseResponseExpansion()
{
  return static_cast<bool>(m_pUseResponseExpansion->GetValue());
}

int Mapper::getParamMinPassThrough()
{
  return static_cast<int>(m_pMinPassThrough->GetValue());
}

double Mapper::getParamOccupancyThreshold()
{
  return static_cast<double>(m_pOccupancyThreshold->GetValue());
}

/* Setters for parameters */
// General Parameters
void Mapper::setParamUseScanMatching(bool b)
{
  m_pUseScanMatching->SetValue((kt_bool)b);
}

void Mapper::setParamUseScanBarycenter(bool b)
{
  m_pUseScanBarycenter->SetValue((kt_bool)b);
}

void Mapper::setParamMinimumTimeInterval(double d)
{
  m_pMinimumTimeInterval->SetValue((kt_double)d);
}

void Mapper::setParamMinimumTravelDistance(double d)
{
  m_pMinimumTravelDistance->SetValue((kt_double)d);
}

void Mapper::setParamMinimumTravelHeading(double d)
{
  m_pMinimumTravelHeading->SetValue((kt_double)d);
}

void Mapper::setParamScanBufferSize(int i)
{
  m_pScanBufferSize->SetValue((kt_int32u)i);
}

void Mapper::setParamScanBufferMaximumScanDistance(double d)
{
  m_pScanBufferMaximumScanDistance->SetValue((kt_double)d);
}

void Mapper::setParamLinkMatchMinimumResponseFine(double d)
{
  m_pLinkMatchMinimumResponseFine->SetValue((kt_double)d);
}

void Mapper::setParamLinkScanMaximumDistance(double d)
{
  m_pLinkScanMaximumDistance->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchMaximumDistance(double d)
{
  m_pLoopSearchMaximumDistance->SetValue((kt_double)d);
}

void Mapper::setParamDoLoopClosing(bool b)
{
  m_pDoLoopClosing->SetValue((kt_bool)b);
}

void Mapper::setParamLoopMatchMinimumChainSize(int i)
{
  m_pLoopMatchMinimumChainSize->SetValue((kt_int32u)i);
}

void Mapper::setParamLoopMatchMaximumVarianceCoarse(double d)
{
  m_pLoopMatchMaximumVarianceCoarse->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamLoopMatchMinimumResponseCoarse(double d)
{
  m_pLoopMatchMinimumResponseCoarse->SetValue((kt_double)d);
}

void Mapper::setParamLoopMatchMinimumResponseFine(double d)
{
  m_pLoopMatchMinimumResponseFine->SetValue((kt_double)d);
}

// Correlation Parameters - Correlation Parameters
void Mapper::setParamCorrelationSearchSpaceDimension(double d)
{
  m_pCorrelationSearchSpaceDimension->SetValue((kt_double)d);
}

void Mapper::setParamCorrelationSearchSpaceResolution(double d)
{
  m_pCorrelationSearchSpaceResolution->SetValue((kt_double)d);
}

void Mapper::setParamCorrelationSearchSpaceSmearDeviation(double d)
{
  m_pCorrelationSearchSpaceSmearDeviation->SetValue((kt_double)d);
}


// Correlation Parameters - Loop Closure Parameters
void Mapper::setParamLoopSearchSpaceDimension(double d)
{
  m_pLoopSearchSpaceDimension->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchSpaceResolution(double d)
{
  m_pLoopSearchSpaceResolution->SetValue((kt_double)d);
}

void Mapper::setParamLoopSearchSpaceSmearDeviation(double d)
{
  m_pLoopSearchSpaceSmearDeviation->SetValue((kt_double)d);
}


// Scan Matcher Parameters
void Mapper::setParamDistanceVariancePenalty(double d)
{
  m_pDistanceVariancePenalty->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamAngleVariancePenalty(double d)
{
  m_pAngleVariancePenalty->SetValue((kt_double)math::Square(d));
}

void Mapper::setParamFineSearchAngleOffset(double d)
{
  m_pFineSearchAngleOffset->SetValue((kt_double)d);
}

void Mapper::setParamCoarseSearchAngleOffset(double d)
{
  m_pCoarseSearchAngleOffset->SetValue((kt_double)d);
}

void Mapper::setParamCoarseAngleResolution(double d)
{
  m_pCoarseAngleResolution->SetValue((kt_double)d);
}

void Mapper::setParamMinimumAnglePenalty(double d)
{
  m_pMinimumAnglePenalty->SetValue((kt_double)d);
}

void Mapper::setParamMinimumDistancePenalty(double d)
{
  m_pMinimumDistancePenalty->SetValue((kt_double)d);
}

void Mapper::setParamUseResponseExpansion(bool b)
{
  m_pUseResponseExpansion->SetValue((kt_bool)b);
}

void Mapper::setParamMinPassThrough(int i)
{
  m_pMinPassThrough->SetValue((kt_int32u)i);
}

void Mapper::setParamOccupancyThreshold(double d)
{
  m_pOccupancyThreshold->SetValue((kt_double)d);
}


void Mapper::Initialize(kt_double rangeThreshold)
{
  if (m_Initialized) {
    return;
  }
  // create sequential scan and loop matcher, update if deserialized

  if (m_pSequentialScanMatcher) {
    delete m_pSequentialScanMatcher;
  }
  m_pSequentialScanMatcher = ScanMatcher::Create(this,
    m_pCorrelationSearchSpaceDimension->GetValue(),
    m_pCorrelationSearchSpaceResolution->GetValue(),
    m_pCorrelationSearchSpaceSmearDeviation->GetValue(),
    rangeThreshold);
  assert(m_pSequentialScanMatcher);

  if (m_Deserialized) {
    m_pMapperSensorManager->SetRunningScanBufferSize(m_pScanBufferSize->GetValue());
    m_pMapperSensorManager->SetRunningScanBufferMaximumDistance(m_pScanBufferMaximumScanDistance->GetValue());

    m_pGraph->UpdateLoopScanMatcher(rangeThreshold);
  } else {
    m_pMapperSensorManager = new MapperSensorManager(m_pScanBufferSize->GetValue(),
      m_pScanBufferMaximumScanDistance->GetValue());

    m_pGraph = new MapperGraph(this, rangeThreshold);
  }

  m_Initialized = true;
}

void Mapper::SaveToFile(const std::string & filename)
{
  printf("Save To File %s \n", filename.c_str());
  std::ofstream ofs(filename.c_str());
  boost::archive::binary_oarchive oa(ofs, boost::archive::no_codecvt);
  oa << BOOST_SERIALIZATION_NVP(*this);
}

void Mapper::LoadFromFile(const std::string & filename)
{
  printf("Load From File %s \n", filename.c_str());
  std::ifstream ifs(filename.c_str());
  boost::archive::binary_iarchive ia(ifs, boost::archive::no_codecvt);
  ia >> BOOST_SERIALIZATION_NVP(*this);
  m_Deserialized = true;
  m_Initialized = false;
}

void Mapper::Reset()
{
  if (m_pSequentialScanMatcher) {
    delete m_pSequentialScanMatcher;
    m_pSequentialScanMatcher = NULL;
  }
  if (m_pGraph) {
    delete m_pGraph;
    m_pGraph = NULL;
  }
  if (m_pMapperSensorManager) {
    delete m_pMapperSensorManager;
    m_pMapperSensorManager = NULL;
  }
  m_Initialized = false;
  m_Deserialized = false;
  while (!m_LocalizationScanVertices.empty()) {
    m_LocalizationScanVertices.pop();
  }
}

kt_bool Mapper::Process(Object *  /*pObject*/)  // NOLINT
{
  return true;
}

kt_bool Mapper::Process(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL || pLaserRangeFinder->Validate(pScan) == false) {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // get last scan
    LocalizedRangeScan * pLastScan = m_pMapperSensorManager->GetLastScan(pScan->GetSensorName());

    // update scans corrected pose based on last correction
    if (pLastScan != NULL) {
      Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
      pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
    }

    // test if scan is outside minimum boundary or if heading is larger then minimum heading
    if (!HasMovedEnough(pScan, pLastScan)) {
      return false;
    }

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
      if (covariance) {
        *covariance = cov;
      }
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

kt_bool Mapper::ProcessAgainstNodesNearBy(LocalizedRangeScan * pScan, kt_bool addScanToLocalizationBuffer, Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL ||
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    Vertex<LocalizedRangeScan> * closetVertex = m_pGraph->FindNearByScan(
      pScan->GetSensorName(), pScan->GetOdometricPose());
    LocalizedRangeScan * pLastScan = NULL;
    if (closetVertex) {
      pLastScan = m_pMapperSensorManager->GetScan(pScan->GetSensorName(),
          closetVertex->GetObject()->GetStateId());
      m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
      m_pMapperSensorManager->AddRunningScan(pLastScan);
      m_pMapperSensorManager->SetLastScan(pLastScan);
    }

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());

    if (covariance) {
      *covariance = cov;
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    Vertex<LocalizedRangeScan> * scan_vertex = NULL;
    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      scan_vertex = m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    if (addScanToLocalizationBuffer) {
      AddScanToLocalizationBuffer(pScan, scan_vertex);
    }

    return true;
  }

  return false;
}

/**
 * 处理用于定位的扫描数据，该函数用于在已构建的地图中进行定位
 * @param pScan 需要处理的激光扫描数据
 * @param covariance 输出参数，返回位姿估计的协方差矩阵
 * @return 处理成功返回true，否则返回false
 */
kt_bool Mapper::ProcessLocalization(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  // 检查输入参数是否有效
  if (pScan == nullptr) {
    return false;
  }

  // 获取激光雷达设备对象
  karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

  // 验证扫描数据的有效性
  if (pLaserRangeFinder == nullptr || pScan == nullptr ||
    pLaserRangeFinder->Validate(pScan) == false)
  {
    return false;
  }

  // 如果尚未初始化，则使用设备的距离阈值进行初始化
  if (m_Initialized == false) {
    // initialize mapper with range threshold from device
    Initialize(pLaserRangeFinder->GetRangeThreshold());
  }

  // 获取同一传感器的上一次扫描数据
  // get last scan
  LocalizedRangeScan * pLastScan = m_pMapperSensorManager->GetLastScan(
    pScan->GetSensorName());

  // 根据上一次扫描的校正位姿更新当前扫描的校正位姿
  // update scans corrected pose based on last correction
  if (pLastScan != nullptr) {
    // 创建从里程计位姿到校正位姿的变换
    Transform lastTransform(pLastScan->GetOdometricPose(),
      pLastScan->GetCorrectedPose());
    // 应用该变换到当前扫描的里程计位姿上，得到初始校正位姿
    pScan->SetCorrectedPose(lastTransform.TransformPose(
        pScan->GetOdometricPose()));
  }

  // 检查当前扫描与上一次扫描之间是否有足够的运动（距离或角度）
  // test if scan is outside minimum boundary
  // or if heading is larger then minimum heading
  if (!HasMovedEnough(pScan, pLastScan)) {
    return false;
  }

  // 初始化协方差矩阵为单位矩阵
  Matrix3 cov;
  cov.SetToIdentity();

  // 如果启用了扫描匹配并且存在上一次扫描，则进行扫描匹配来优化位姿
  // correct scan (if not first scan)
  if (m_pUseScanMatching->GetValue() && pLastScan != nullptr) {
    Pose2 bestPose;
    // 使用顺序扫描匹配器将当前扫描与运行扫描缓冲区中的扫描进行匹配
    m_pSequentialScanMatcher->MatchScan(pScan,
      m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
      bestPose,
      cov);
    // 设置扫描的最佳估计位姿
    pScan->SetSensorPose(bestPose);
    // 如果提供了协方差指针，则返回协方差矩阵
    if (covariance) {
      *covariance = cov;
    }
  }

  // 将扫描添加到扫描管理器中并分配唯一ID
  // add scan to buffer and assign id
  m_pMapperSensorManager->AddScan(pScan);

  // 用于存储图中顶点的指针
  Vertex<LocalizedRangeScan> * scan_vertex = nullptr;

  // 如果启用了扫描匹配
  if (m_pUseScanMatching->GetValue()) {
    // add to graph
    // 将扫描添加为图中的一个顶点
    scan_vertex = m_pGraph->AddVertex(pScan);
    // 为该扫描添加边（约束）
    m_pGraph->AddEdges(pScan, cov);

    // 将扫描添加到运行扫描缓冲区中
    m_pMapperSensorManager->AddRunningScan(pScan);

    // 如果启用了回环检测
    if (m_pDoLoopClosing->GetValue()) {
      // 获取所有传感器名称
      std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();
      // 对每个传感器尝试进行回环检测
      const_forEach(std::vector<Name>, &deviceNames)
      {
        m_pGraph->TryCloseLoop(pScan, *iter);
      }
    }
  }

  // 设置当前扫描为该传感器的最后一次扫描
  m_pMapperSensorManager->SetLastScan(pScan);

  // 将扫描添加到定位缓冲区中（用于定位模式下的特殊处理）
  AddScanToLocalizationBuffer(pScan, scan_vertex);

  return true;
}


void Mapper::AddScanToLocalizationBuffer(LocalizedRangeScan * pScan, Vertex <LocalizedRangeScan> * scan_vertex)
{
  // generate the info to store and later decay, outside of dataset
  LocalizationScanVertex lsv;
  lsv.scan = pScan;
  lsv.vertex = scan_vertex;
  m_LocalizationScanVertices.push(lsv);

  if (m_LocalizationScanVertices.size() > getParamScanBufferSize()) {
    LocalizationScanVertex & oldLSV = m_LocalizationScanVertices.front();
    RemoveNodeFromGraph(oldLSV.vertex);

    // delete node and scans
    // free hat!
    // No need to delete from m_scans as those pointers will be freed memory
    oldLSV.vertex->RemoveObject();
    m_pMapperSensorManager->RemoveScan(oldLSV.scan);
    if (oldLSV.scan) {
      delete oldLSV.scan;
      oldLSV.scan = NULL;
    }

    m_LocalizationScanVertices.pop();
  }
}

void Mapper::ClearLocalizationBuffer()
{
  while (!m_LocalizationScanVertices.empty())
  {
    LocalizationScanVertex& oldLSV = m_LocalizationScanVertices.front();
    RemoveNodeFromGraph(oldLSV.vertex);
    oldLSV.vertex->RemoveObject();
    m_pMapperSensorManager->RemoveScan(oldLSV.scan);
    if (oldLSV.scan)
    {
      delete oldLSV.scan;
      oldLSV.scan = NULL;
    }

    m_LocalizationScanVertices.pop();
  }

  std::vector<Name> names = m_pMapperSensorManager->GetSensorNames();
  for (uint i = 0; i != names.size(); i++)
  {
    m_pMapperSensorManager->ClearRunningScans(names[i]);
    m_pMapperSensorManager->ClearLastScan(names[i]);
  }

  return;
}

kt_bool Mapper::RemoveNodeFromGraph(Vertex<LocalizedRangeScan> * vertex_to_remove)
{
  // 1) delete edges in adjacent vertices, graph, and optimizer
  std::vector<Vertex<LocalizedRangeScan> *> adjVerts =
    vertex_to_remove->GetAdjacentVertices();
  for (int i = 0; i != adjVerts.size(); i++) {
    std::vector<Edge<LocalizedRangeScan> *> adjEdges = adjVerts[i]->GetEdges();
    bool found = false;
    for (int j = 0; j != adjEdges.size(); j++) {
      if (adjEdges[j]->GetTarget() == vertex_to_remove ||
        adjEdges[j]->GetSource() == vertex_to_remove)
      {
        adjVerts[i]->RemoveEdge(j);
        m_pScanOptimizer->RemoveConstraint(
          adjEdges[j]->GetSource()->GetObject()->GetUniqueId(),
          adjEdges[j]->GetTarget()->GetObject()->GetUniqueId());
        std::vector<Edge<LocalizedRangeScan> *> edges = m_pGraph->GetEdges();
        std::vector<Edge<LocalizedRangeScan> *>::iterator edgeGraphIt =
          std::find(edges.begin(), edges.end(), adjEdges[j]);

        if (edgeGraphIt == edges.end()) {
          std::cout << "Edge not found in graph to remove!" << std::endl;
          continue;
        }

        int posEdge = edgeGraphIt - edges.begin();
        m_pGraph->RemoveEdge(posEdge);   // remove from graph
        delete *edgeGraphIt;   // free hat!
        *edgeGraphIt = NULL;
        found = true;
      }
    }
    if (!found) {
      std::cout << "Failed to find any edge in adj. vertex" <<
        " with a matching vertex to current!" << std::endl;
    }
  }

  // 2) delete vertex from optimizer
  m_pScanOptimizer->RemoveNode(vertex_to_remove->GetObject()->GetUniqueId());

  // 3) delete from vertex map
  std::map<Name, std::map<int, Vertex<LocalizedRangeScan> *>>
  vertexMap = m_pGraph->GetVertices();
  std::map<int, Vertex<LocalizedRangeScan> *> graphVertices =
    vertexMap[vertex_to_remove->GetObject()->GetSensorName()];
  std::map<int, Vertex<LocalizedRangeScan> *>::iterator
    vertexGraphIt = graphVertices.find(vertex_to_remove->GetObject()->GetStateId());
  if (vertexGraphIt != graphVertices.end()) {
    m_pGraph->RemoveVertex(vertex_to_remove->GetObject()->GetSensorName(),
      vertexGraphIt->second->GetObject()->GetStateId());
  } else {
    std::cout << "Vertex not found in graph to remove!" << std::endl;
    return false;
  }

  return true;
}

kt_bool Mapper::ProcessAgainstNode(
  LocalizedRangeScan * pScan,
  const int & nodeId,
  Matrix3 * covariance)
{
  if (pScan != NULL) {
    karto::LaserRangeFinder * pLaserRangeFinder = pScan->GetLaserRangeFinder();

    // validate scan
    if (pLaserRangeFinder == NULL || pScan == NULL ||
      pLaserRangeFinder->Validate(pScan) == false)
    {
      return false;
    }

    if (m_Initialized == false) {
      // initialize mapper with range threshold from device
      Initialize(pLaserRangeFinder->GetRangeThreshold());
    }

    // If we're matching against a node from an older mapping session
    // lets get the first scan as the last scan and populate running scans
    // with the first few from that run as well.
    LocalizedRangeScan * pLastScan =
      m_pMapperSensorManager->GetScan(pScan->GetSensorName(), nodeId);
    m_pMapperSensorManager->ClearRunningScans(pScan->GetSensorName());
    m_pMapperSensorManager->AddRunningScan(pLastScan);
    m_pMapperSensorManager->SetLastScan(pLastScan);

    Matrix3 cov;
    cov.SetToIdentity();

    // correct scan (if not first scan)
    if (m_pUseScanMatching->GetValue() && pLastScan != NULL) {
      Pose2 bestPose;
      m_pSequentialScanMatcher->MatchScan(pScan,
        m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),
        bestPose,
        cov);
      pScan->SetSensorPose(bestPose);
    }

    pScan->SetOdometricPose(pScan->GetCorrectedPose());
    if (covariance) {
      *covariance = cov;
    }

    // add scan to buffer and assign id
    m_pMapperSensorManager->AddScan(pScan);

    if (m_pUseScanMatching->GetValue()) {
      // add to graph
      m_pGraph->AddVertex(pScan);
      m_pGraph->AddEdges(pScan, cov);

      m_pMapperSensorManager->AddRunningScan(pScan);

      if (m_pDoLoopClosing->GetValue()) {
        std::vector<Name> deviceNames =
          m_pMapperSensorManager->GetSensorNames();
        const_forEach(std::vector<Name>, &deviceNames)
        {
          m_pGraph->TryCloseLoop(pScan, *iter);
        }
      }
    }

    m_pMapperSensorManager->SetLastScan(pScan);

    return true;
  }

  return false;
}

kt_bool Mapper::ProcessAtDock(LocalizedRangeScan * pScan, Matrix3 * covariance)
{
  // Special case of processing against node where node is the starting point
  return ProcessAgainstNode(pScan, 0, covariance);
}

/**
 * Is the scan sufficiently far from the last scan?
 * @param pScan
 * @param pLastScan
 * @return true if the scans are sufficiently far
 */
kt_bool Mapper::HasMovedEnough(LocalizedRangeScan * pScan, LocalizedRangeScan * pLastScan) const
{
  // test if first scan
  if (pLastScan == NULL) {
    return true;
  }

  // test if enough time has passed
  kt_double timeInterval = pScan->GetTime() - pLastScan->GetTime();
  if (timeInterval >= m_pMinimumTimeInterval->GetValue()) {
    return true;
  }

  Pose2 lastScannerPose = pLastScan->GetSensorAt(pLastScan->GetOdometricPose());
  Pose2 scannerPose = pScan->GetSensorAt(pScan->GetOdometricPose());

  // test if we have turned enough
  kt_double deltaHeading = math::NormalizeAngle(
    scannerPose.GetHeading() - lastScannerPose.GetHeading());
  if (fabs(deltaHeading) >= m_pMinimumTravelHeading->GetValue()) {
    return true;
  }

  // test if we have moved enough
  kt_double squaredTravelDistance = lastScannerPose.GetPosition().SquaredDistance(
    scannerPose.GetPosition());
  if (squaredTravelDistance >= math::Square(m_pMinimumTravelDistance->GetValue()) - KT_TOLERANCE) {
    return true;
  }

  return false;
}

/**
 * Gets all the processed scans
 * @return all scans
 */
const LocalizedRangeScanVector Mapper::GetAllProcessedScans() const
{
  LocalizedRangeScanVector allScans;

  if (m_pMapperSensorManager != NULL) {
    allScans = m_pMapperSensorManager->GetAllScans();
  }

  return allScans;
}

/**
 * Adds a listener
 * @param pListener
 */
void Mapper::AddListener(MapperListener * pListener)
{
  m_Listeners.push_back(pListener);
}

/**
 * Removes a listener
 * @param pListener
 */
void Mapper::RemoveListener(MapperListener * pListener)
{
  std::vector<MapperListener *>::iterator iter = std::find(m_Listeners.begin(),
      m_Listeners.end(), pListener);
  if (iter != m_Listeners.end()) {
    m_Listeners.erase(iter);
  }
}

void Mapper::FireInfo(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    (*iter)->Info(rInfo);
  }
}

void Mapper::FireDebug(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperDebugListener * pListener = dynamic_cast<MapperDebugListener *>(*iter);

    if (pListener != NULL) {
      pListener->Debug(rInfo);
    }
  }
}

void Mapper::FireLoopClosureCheck(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->LoopClosureCheck(rInfo);
    }
  }
}

void Mapper::FireBeginLoopClosure(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->BeginLoopClosure(rInfo);
    }
  }
}

void Mapper::FireEndLoopClosure(const std::string & rInfo) const
{
  const_forEach(std::vector<MapperListener *>, &m_Listeners)
  {
    MapperLoopClosureListener * pListener = dynamic_cast<MapperLoopClosureListener *>(*iter);

    if (pListener != NULL) {
      pListener->EndLoopClosure(rInfo);
    }
  }
}

void Mapper::SetScanSolver(ScanSolver * pScanOptimizer)
{
  m_pScanOptimizer = pScanOptimizer;
}

ScanSolver * Mapper::getScanSolver()
{
  return m_pScanOptimizer;
}

MapperGraph * Mapper::GetGraph() const
{
  return m_pGraph;
}

ScanMatcher * Mapper::GetSequentialScanMatcher() const
{
  return m_pSequentialScanMatcher;
}

ScanMatcher * Mapper::GetLoopScanMatcher() const
{
  return m_pGraph->GetLoopScanMatcher();
}
}  // namespace karto
BOOST_CLASS_EXPORT(karto::BreadthFirstTraversal<karto::LocalizedRangeScan>)
