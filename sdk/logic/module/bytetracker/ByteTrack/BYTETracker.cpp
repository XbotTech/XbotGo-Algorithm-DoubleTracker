#include "BYTETracker.h"
#include <fstream>
#include <sys/time.h>

BYTETracker::BYTETracker(const int &frame_rate,
                                     const int &track_buffer,
                                     const float &track_thresh,
                                     const float &high_thresh,
                                     const float &match_thresh)
{
	this->track_thresh = track_thresh;
	this->high_thresh = high_thresh;
	this->match_thresh = match_thresh;

	frame_id = 0;
	max_time_lost = int(frame_rate / 30.0 * track_buffer);
	// std::cout << "Init ByteTrack!" << std::endl;
}

BYTETracker::~BYTETracker()
{
}

void BYTETracker::update(std::vector<STrack> & output_stracks, const std::vector<Object>& objects)
{
	output_stracks.clear();
	////////////////// Step 1: Get detections //////////////////
	this->frame_id++;
	std::vector<STrack> activated_stracks;
	std::vector<STrack> refind_stracks;
	std::vector<STrack> removed_stracks;
	std::vector<STrack> lost_stracks;
	std::vector<STrack> detections;
	std::vector<STrack> detections_low;

	std::vector<STrack> detections_cp;
	std::vector<STrack> tracked_stracks_swap;
	std::vector<STrack> resa, resb;

	std::vector<STrack*> unconfirmed;
	std::vector<STrack*> tracked_stracks;
	std::vector<STrack*> strack_pool;
	std::vector<STrack*> r_tracked_stracks;

	if (objects.size() > 0)
	{
		for (int i = 0; i < objects.size(); i++)
		{
			std::vector<float> tlbr_;
			tlbr_.resize(4);
			tlbr_[0] = objects[i].rect.x;
			tlbr_[1] = objects[i].rect.y;
			tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
			tlbr_[3] = objects[i].rect.y + objects[i].rect.height;

			float score = objects[i].prob;

			STrack strack(STrack::tlbr_to_tlwh(tlbr_), score);
			if (score >= track_thresh)
			{
				detections.push_back(strack);
			}
			else
			{
				detections_low.push_back(strack);
			}
			
		}
	}

	// Add newly detected tracklets to tracked_stracks
	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (!this->tracked_stracks[i].is_activated)
			unconfirmed.push_back(&this->tracked_stracks[i]);
		else
			tracked_stracks.push_back(&this->tracked_stracks[i]);
	}

	////////////////// Step 2: First association, with IoU //////////////////
	joint_stracks(strack_pool, tracked_stracks, this->lost_stracks);

	
	STrack::multi_predict(strack_pool, this->kalman_filter);

	std::vector<std::vector<float> > dists;
	int dist_size = 0, dist_size_size = 0;
	iou_distance(dists, strack_pool, detections, dist_size, dist_size_size);

	std::vector<std::vector<int> > matches;
	std::vector<int> u_track, u_detection;
	linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

	for (int i = 0; i < matches.size(); i++)
	{
		STrack *track = strack_pool[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	////////////////// Step 3: Second association, using low score dets //////////////////
	for (int i = 0; i < u_detection.size(); i++)
	{
		detections_cp.push_back(detections[u_detection[i]]);
	}
	detections.clear();
	detections.assign(detections_low.begin(), detections_low.end());
	
	for (int i = 0; i < u_track.size(); i++)
	{
		if (strack_pool[u_track[i]]->state == TrackState::Tracked)
		{
			r_tracked_stracks.push_back(strack_pool[u_track[i]]);
		}
	}

	dists.clear();
	iou_distance(dists, r_tracked_stracks, detections, dist_size, dist_size_size);

	matches.clear();
	u_track.clear();
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

	for (int i = 0; i < matches.size(); i++)
	{
		STrack *track = r_tracked_stracks[matches[i][0]];
		STrack *det = &detections[matches[i][1]];
		if (track->state == TrackState::Tracked)
		{
			track->update(*det, this->frame_id);
			activated_stracks.push_back(*track);
		}
		else
		{
			track->re_activate(*det, this->frame_id, false);
			refind_stracks.push_back(*track);
		}
	}

	for (int i = 0; i < u_track.size(); i++)
	{
		STrack *track = r_tracked_stracks[u_track[i]];
		if (track->state != TrackState::Lost)
		{
			track->mark_lost();
			lost_stracks.push_back(*track);
		}
	}

	// Deal with unconfirmed tracks, usually tracks with only one beginning frame
	detections.clear();
	detections.assign(detections_cp.begin(), detections_cp.end());

	dists.clear();
	iou_distance(dists, unconfirmed, detections, dist_size, dist_size_size);

	matches.clear();
	std::vector<int> u_unconfirmed;
	u_detection.clear();
	linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

	for (int i = 0; i < matches.size(); i++)
	{
		unconfirmed[matches[i][0]]->update(detections[matches[i][1]], this->frame_id);
		activated_stracks.push_back(*unconfirmed[matches[i][0]]);
	}

	for (int i = 0; i < u_unconfirmed.size(); i++)
	{
		STrack *track = unconfirmed[u_unconfirmed[i]];
		track->mark_removed();
		removed_stracks.push_back(*track);
	}

	////////////////// Step 4: Init new stracks //////////////////
	for (int i = 0; i < u_detection.size(); i++)
	{
		STrack *track = &detections[u_detection[i]];
		if (track->score < this->high_thresh)
			continue;
		track->activate(this->kalman_filter, this->frame_id);
		activated_stracks.push_back(*track);
	}

	////////////////// Step 5: Update state //////////////////
	// for (int i = 0; i < this->lost_stracks.size(); i++)
	// {
	// 	if (this->frame_id - this->lost_stracks[i].end_frame() > this->max_time_lost)
	// 	{
	// 		this->lost_stracks[i].mark_removed();
	// 		removed_stracks.push_back(this->lost_stracks[i]);
	// 	}
	// }

	// 对丢失时间大于指定帧数的轨迹进行删除
    for (auto it = this->lost_stracks.begin(); it != this->lost_stracks.end();)
	{
        if (this->frame_id - it->end_frame() > this->max_time_lost)
		{
            it->mark_removed();
            removed_stracks.push_back(*it);
            it = this->lost_stracks.erase(it); // 同时删除这些过期对象
        } else {
            ++it;
        }
    }
	// 组合：当前跟踪轨迹 ∪ 重新跟踪轨迹 为已跟踪轨迹
	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].state == TrackState::Tracked)
		{
			tracked_stracks_swap.push_back(this->tracked_stracks[i]);
		}
	}
	this->tracked_stracks.clear();
	this->tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

	this->tracked_stracks = joint_stracks(this->tracked_stracks, activated_stracks);
	this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

	//std::cout << activated_stracks.size() << std::endl;

	// 组合:((丢失轨迹 - 已跟踪轨迹)U 当前丟失轨迹)-已删除轨迹 为 丢失轨迹
	sub_stracks(this->lost_stracks, this->lost_stracks, this->tracked_stracks);
	for (int i = 0; i < lost_stracks.size(); i++)  // 0.2s
	{
		this->lost_stracks.push_back(lost_stracks[i]);
	}

	sub_stracks(this->lost_stracks, this->lost_stracks, removed_stracks);
	// 组合:已删除轨迹 U当前删除轨迹
	// for (int i = 0; i < removed_stracks.size(); i++)
	// {
	// 	this->removed_stracks.push_back(removed_stracks[i]);
	// }
	// 输出去重
	remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);

	this->tracked_stracks.clear();
	this->tracked_stracks.assign(resa.begin(), resa.end());
	this->lost_stracks.clear();
	this->lost_stracks.assign(resb.begin(), resb.end());
	// 对已激活(跟踪)轨迹进行输出
	for (int i = 0; i < this->tracked_stracks.size(); i++)
	{
		if (this->tracked_stracks[i].is_activated)
		{
			output_stracks.push_back(this->tracked_stracks[i]);
		}
	}

	//return output_stracks;
}